// The board's own timing on every feedback line, for Perception's recordings
// (D:\Perception docs/rover-schema-v2.json and rover-drive-spec.md, "Schema,
// version 2", agreed 2026-10-31). Added by the rover project; not Waveshare's.
//
//   ts  micros() just before the line is written
//   bi  this boot's id: a counter kept in NVS, raised once at every boot
//   ti  micros() when the gyro packet behind gx..gz was read from the DMP FIFO
//   n   gyro-bearing packets accepted since boot, that one included
//   gc  that packet's 16-bit DMP footer, raw
//   ln  loss events counted since boot when that packet was accepted
//
// ti, n, gc, ln and gx..gz are one snapshot, changed together and only when a
// gyro-bearing packet is accepted. A packet is accepted only with the FIFO
// overflow status (INT_STATUS_2, cleared by reading) clear both before its read
// starts and after it ends, and the read itself complete. Anything else -- an
// overflow seen, a status read that failed, a packet read that failed or stopped
// part way -- is a loss event: the packet is discarded, the snapshot left as it
// was, the FIFO reset, and nothing accepted until the reset has succeeded. Every
// FIFO or DMP reset this firmware makes counts as a loss event, the two at boot
// included. Nothing else in the firmware reads INT_STATUS_2.

#include <Preferences.h>

uint32_t timingBootId = 0;
uint32_t timingLoss = 0;
bool timingResetPending = false;

bool timingHaveSample = false;
uint32_t timingReadUs = 0;
uint32_t timingSeq = 0;
uint16_t timingFooter = 0;
uint32_t timingSampleLoss = 0;

void timingBootInit() {
  Preferences prefs;
  prefs.begin("ugvtiming", false);
  timingBootId = prefs.getUInt("boot", 0) + 1;
  prefs.putUInt("boot", timingBootId);
  prefs.end();
}

// Reads INT_STATUS_2; true when the read succeeded, with *clear saying whether no
// FIFO overflow was flagged.
bool timingOverflowStatus(bool *clear) {
  uint8_t status = 0;
  if (myICM.setBank(0) != ICM_20948_Stat_Ok) return false;
  if (myICM.read(AGB0_REG_INT_STATUS_2, &status, 1) != ICM_20948_Stat_Ok) return false;
  *clear = (status & 0x1F) == 0;
  return true;
}

// One pass's read of the DMP FIFO, replacing the vendor's: at most one packet a
// pass, as before ("one_per_pass").
void timingReadIMU() {
  if (timingResetPending) {
    if (myICM.resetFIFO() != ICM_20948_Stat_Ok) return;
    timingResetPending = false;
    return;  // the pass after a reset starts clean
  }
  bool clearBefore = false, clearAfter = false;
  bool statusBefore = timingOverflowStatus(&clearBefore);
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  ICM_20948_Status_e read = myICM.status;
  uint32_t readUs = micros();
  bool statusAfter = timingOverflowStatus(&clearAfter);

  bool complete = (read == ICM_20948_Stat_Ok) || (read == ICM_20948_Stat_FIFOMoreDataAvail);
  bool empty = (read == ICM_20948_Stat_FIFONoDataAvail);
  if (!statusBefore || !statusAfter || !clearBefore || !clearAfter || !(complete || empty)) {
    timingLoss++;
    timingResetPending = true;
    return;
  }
  if (!complete) return;

  if ((data.header & DMP_header_bitmap_Accel) > 0) {
    ax = data.Raw_Accel.Data.X;
    ay = data.Raw_Accel.Data.Y;
    az = data.Raw_Accel.Data.Z;
  }
  if ((data.header & DMP_header_bitmap_Compass) > 0) {
    mx = data.Compass.Data.X;
    my = data.Compass.Data.Y;
    mz = data.Compass.Data.Z;
  }
  if ((data.header & DMP_header_bitmap_Gyro) > 0) {
    gx = data.Raw_Gyro.Data.X;
    gy = data.Raw_Gyro.Data.Y;
    gz = data.Raw_Gyro.Data.Z;
    timingReadUs = readUs;
    timingSeq++;
    timingFooter = data.Footer;
    timingSampleLoss = timingLoss;
    timingHaveSample = true;
  }
}

// The timing keys on a feedback line; ts is set by the caller just before writing.
void timingAddToFeedback(JsonDocument &doc) {
  doc["bi"] = timingBootId;
  if (timingHaveSample) {
    doc["ti"] = timingReadUs;
    doc["n"] = timingSeq;
    doc["gc"] = timingFooter;
    doc["ln"] = timingSampleLoss;
  } else {
    doc.remove("gx");
    doc.remove("gy");
    doc.remove("gz");
  }
}
