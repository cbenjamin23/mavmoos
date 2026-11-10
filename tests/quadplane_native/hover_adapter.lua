-- Owned fixed-motor QuadPlane SITL target adapter for stock Plane 4.6.3.
if param:get("SIM_SPEEDUP") ~= 1 then
    return
end

local channel, message_id = 0, 385
local bridge_system, bridge_component = 245, 191
local guided, qloiter = 15, 19
local marker = "MQH1"
local challenge, previous_challenge, last_accepted, lifetime = 0, 0, 0, 0
local owns_hover = false
local last_mode, eligible_since = nil, nil

mavlink:init(1, 4)
assert(mavlink:register_rx_msgid(message_id))

local function hover_position(mode)
    if (mode ~= guided and mode ~= qloiter) or not arming:is_armed() or
            not vehicle:get_likely_flying() or vehicle:has_ekf_failsafed() or
            not quadplane:in_vtol_mode() or quadplane:in_assisted_flight() or
            param:get("SIM_SPEEDUP") ~= 1 or param:get("Q_GUIDED_MODE") ~= 1 or param:get("FENCE_ENABLE") ~= 0 or
            param:get("FENCE_AUTOENABLE") ~= 0 then
        return nil
    end
    local velocity = ahrs:get_velocity_NED()
    local position = ahrs:get_position()
    if not velocity or not position or not position:change_alt_frame(0) or
            math.abs(velocity:z()) > 2 or velocity:x()^2 + velocity:y()^2 > 25 then
        return nil
    end
    local home = ahrs:get_home()
    if not home or not home:change_alt_frame(0) or position:alt() - home:alt() < 2500 or
            position:alt() - home:alt() > 9000 then
        return nil
    end
    return position, home
end

local function offer(now, mode)
    local data = string.pack("<c4I4BB", marker, now, mode == guided and 1 or 3, 1)
    local packet = string.pack("<I2BBBc128", 0, bridge_system, bridge_component, #data, data)
    if mavlink:send_chan(channel, message_id, packet) then
        previous_challenge, challenge = challenge, now
    end
end

local function accept(msg, incoming, now, position, home)
    if incoming ~= channel or string.byte(msg, 3) ~= 0xFD then
        return
    end
    local length = string.byte(msg, 4)
    local system, component, id = string.unpack("<BBI3", msg, 8)
    if system ~= bridge_system or component ~= bridge_component or id ~= message_id or length ~= 29 then
        return
    end
    local type_id, to_system, to_component, size = string.unpack("<I2BBB", msg, 13)
    local magic, token, action, ttl, lat, lon, alt, end_byte = string.unpack("<c4I4BI2i4i4i4B", msg, 18)
    if type_id ~= 0 or to_system ~= 1 or to_component ~= 1 or size ~= 24 or magic ~= marker or
            (token ~= challenge and token ~= previous_challenge) or token <= last_accepted or action ~= 2 or end_byte ~= 1 or ttl == 0 or ttl > 750 or
            now < token or now - token >= ttl or
            math.abs(lat) >= 900000000 or math.abs(lon) > 1800000000 or
            alt - home:alt() < 2500 or alt - home:alt() > 9000 or
            math.abs(alt - position:alt()) > 300 then
        return
    end
    local current_target = vehicle:get_target_location()
    if not current_target or current_target:get_alt_frame() ~= 0 then
        return
    end
    local desired = current_target:copy()
    desired:lat(lat)
    desired:lng(lon)
    desired:alt(alt)
    if position:get_distance(desired) > 8 or vehicle:get_mode() ~= guided or
            not quadplane:in_vtol_mode() or quadplane:in_assisted_flight() or
            not vehicle:update_target_location(current_target, desired) then
        return
    end
    last_accepted = token
    owns_hover = true
    lifetime = token + ttl
end

local function update()
    local now = millis():toint()
    local mode = vehicle:get_mode()
    local position, home = hover_position(mode)
    if mode ~= last_mode then
        last_mode = mode
        eligible_since = nil
        challenge, previous_challenge = 0, 0
    end
    if mode ~= guided or not arming:is_armed() then
        owns_hover = false
    end
    if not now or now <= 0 or not position then
        eligible_since = nil
        challenge, previous_challenge = 0, 0
    elseif not eligible_since then
        eligible_since = now
    end
    if owns_hover and (not now or now >= lifetime or not position) then
        owns_hover = false
        challenge, previous_challenge = 0, 0
        if vehicle:get_mode() == guided and arming:is_armed() and quadplane:in_vtol_mode() and
                not quadplane:in_assisted_flight() then
            vehicle:set_mode(qloiter)
        end
        position = nil
    end
    for _ = 1, 4 do
        local msg, incoming = mavlink:receive_chan()
        if not msg then
            break
        end
        if position and eligible_since and mode == guided and now - eligible_since >= 160 then
            accept(msg, incoming, now, position, home)
        end
    end
    -- The previous token can still be in transit during rollover. Acceptance
    -- remains monotonic and the lease starts at the original token time.
    if position and eligible_since and now - eligible_since >= 160 and now - challenge >= 120 then
        offer(now, mode)
    end
    return update, 40
end

return update, 40
