#pragma once

#include <cstdint>
#include <string>

namespace mavmoos::runtime {

// Trusted optional service for the owned payload bench. The host chooses the
// wire command, target, deadline and final permission for each operation.
enum class PayloadAction { CameraCaptureOne, GimbalClaim, GimbalPoint, GimbalRelease };

struct PayloadRequest {
  std::string id, operation_id;
  PayloadAction action=PayloadAction::CameraCaptureOne;
  double expires=0, operation_expires=0;
  uint32_t capture_sequence=0;
  float pitch_deg=0, yaw_deg=0;
};

class PayloadService {
public:
  virtual ~PayloadService()=default;
  virtual bool payloadReady(PayloadAction) const=0;
  virtual bool payload(const PayloadRequest&)=0;
  virtual void cancelPayload(const std::string& operation_id)=0;
  virtual void finishPayload(const std::string& operation_id)=0;
};

class PayloadListener {
public:
  virtual ~PayloadListener()=default;
  virtual void onPayloadSent(const PayloadRequest&,double sent_at,bool accepted)=0;
};

} // namespace mavmoos::runtime
