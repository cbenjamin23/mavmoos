-- Check the exact onboard script with simulated bindings and a deterministic clock.
local clock, mode, armed, fence_enabled, sim_speed = 100, 19, false, 0, nil
local writes, holds, registrations = 0, 0, 0
local inbox, offers = {}, {}

local function location(x, y, z)
    local result = {}
    function result:lat(value) if value then x = value end return x end
    function result:lng(value) if value then y = value end return y end
    function result:alt(value) if value then z = value end return z end
    function result:get_alt_frame() return 0 end
    function result:change_alt_frame() return true end
    function result:copy() return location(x, y, z) end
    function result:get_distance(other)
        return math.sqrt((x - other:lat())^2 + (y - other:lng())^2) * .011
    end
    return result
end

local position, home = location(100, 100, 6500), location(100, 100, 1000)
local target = position:copy()
millis = function() return {toint = function() return clock end} end
arming = {is_armed = function() return armed end}
quadplane = {in_vtol_mode = function() return mode == 15 or mode == 19 end,
             in_assisted_flight = function() return false end}
param = {get = function(_, key)
    return ({SIM_SPEEDUP=sim_speed, Q_GUIDED_MODE=1, FENCE_ENABLE=fence_enabled, FENCE_AUTOENABLE=0})[key]
end}
ahrs = {get_position = function() return position:copy() end,
        get_home = function() return home:copy() end,
        get_velocity_NED = function()
            local zero = function() return 0 end
            return {x = zero, y = zero, z = zero}
        end}
vehicle = {get_mode = function() return mode end, get_likely_flying = function() return true end,
           has_ekf_failsafed = function() return false end,
           get_target_location = function() return target end,
           update_target_location = function(_, old, new)
               assert(old == target)
               target = new
               writes = writes + 1
               return true
           end,
           set_mode = function(_, selected)
               assert(mode == 15 and selected == 19)
               mode = selected
               holds = holds + 1
               return true
           end}
mavlink = {init = function(_, ids, depth)
               assert(ids == 1 and depth == 4)
               registrations = registrations + 1
           end,
           register_rx_msgid = function(_, id) return id == 385 end,
           send_chan = function(_, channel, id, packet)
               assert(channel == 0 and id == 385)
               local kind, system, component, length, marker, token, action, sentinel =
                   string.unpack("<I2BBBc4I4BB", packet)
               assert(kind == 0 and system == 245 and component == 191 and length == 10 and
                      marker == "MQH1" and sentinel == 1)
               offers[#offers + 1] = {token = token, action = action}
               return true
           end,
           receive_chan = function()
               local row = table.remove(inbox, 1)
               if row then return row.packet, row.channel end
           end}

local function send(token, ttl, x, source, channel, destination)
    local payload = string.pack("<I2BBBc4I4BI2i4i4i4B", 0, 1, destination or 1, 24, "MQH1",
                                token, 2, ttl, x, 100, 6500, 1)
    local header = string.pack("<I2BBBBBBBI3", 0, 0xfd, #payload, 0, 0, 0, source or 245, 191, 385)
    inbox[#inbox + 1] = {packet = header .. payload, channel = channel or 0}
end

local load_adapter = assert(loadfile(arg[1]))
assert(load_adapter() == nil and registrations == 0)
sim_speed = 2
assert(load_adapter() == nil and registrations == 0)
sim_speed = 1
local update = load_adapter()
assert(registrations == 1)
local function tick(time)
    clock = time
    assert(update() == update)
end

tick(100)
tick(300)
assert(#offers == 0 and writes == 0)
armed = true
tick(400)
tick(560)
assert(offers[#offers].action == 3)
send(560, 700, 200)
tick(580)
assert(writes == 0)
mode = 15
tick(600)
tick(760)
assert(offers[#offers].action == 1)
send(760, 700, 200, 249)
send(760, 700, 200, 245, 2)
send(760, 700, 200, 245, 0, 2)
send(760, 1, 200)
tick(780)
assert(writes == 0)
send(760, 700, 200)
send(760, 700, 300)
tick(800)
assert(writes == 1 and target:lat() == 200)
mode = 5
tick(840)
tick(1480)
assert(holds == 0 and mode == 5)
mode = 15
tick(1520)
tick(1680)
send(760, 700, 300)
send(1680, 700, 400)
tick(1700)
assert(writes == 2 and target:lat() == 400)
tick(2379)
assert(holds == 0)
tick(2380)
assert(holds == 1 and mode == 19)
fence_enabled = 1
local existing = #offers
tick(2420)
tick(2800)
assert(#offers == existing)
print("PASS QuadPlane adapter address, challenge replay, original expiry and pilot ownership")
