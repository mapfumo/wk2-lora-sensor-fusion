# Week 2 - Remaining Tasks

**Status**: 98% Complete - Field testing remains

---

## Range Testing (Day 7)

### Prerequisites
- [ ] Arrange portable power sources for both nodes
  - Node 1: Battery pack or portable power supply
  - Node 2: USB power bank (for probe-rs monitoring)
- [ ] Plan field trip location (open area with minimal obstacles)
- [ ] Prepare measuring tools (GPS/maps for distance verification)

### Test Plan

**Objective**: Validate LoRa communication range and measure signal quality degradation with distance.

**Target**: Packet delivery >95% at 500m (per Phase 1 success criteria)

#### Test Distances
- [ ] **1m baseline** - Record RSSI, SNR, packet loss (expect 100% success)
- [ ] **10m** - Record RSSI, SNR, packet loss
- [ ] **50m** - Record RSSI, SNR, packet loss
- [ ] **100m** - Record RSSI, SNR, packet loss
- [ ] **200m** - Record RSSI, SNR, packet loss (optional)
- [ ] **500m** - Record RSSI, SNR, packet loss (PRIMARY GOAL)

#### Test Procedure
1. Power up both nodes at starting position
2. Verify communication at 1m baseline
3. Move Node 1 to each test distance (keep Node 2 stationary with laptop)
4. At each distance:
   - Let 20-30 packets transmit (2-5 minutes)
   - Record RSSI values from Node 2 logs
   - Record SNR values from Node 2 logs
   - Count total packets transmitted vs received
   - Calculate packet loss percentage
5. Note environmental conditions:
   - Indoor vs outdoor
   - Weather (if outdoor)
   - Obstacles between nodes
   - Antenna orientation

#### Data Collection Template

```
Distance: XXm
Environment: Indoor/Outdoor, [conditions]
Packets Transmitted: XX
Packets Received: XX
Packet Loss: XX%
RSSI Range: -XX to -XX dBm
SNR Range: XX to XX dB
Notes: [any observations]
```

#### Current Baseline (Close Range)
```
Distance: <5m (estimated)
Packets Transmitted: 4
Packets Received: 4
Packet Loss: 0%
RSSI Range: -20 to -29 dBm
SNR Range: 12 to 13 dB
Notes: All packets clean, no corruption
```

---

## Documentation Tasks

### README Update
- [ ] Add "Range Testing Results" section to README.md
- [ ] Include data table with distance vs RSSI/SNR/packet loss
- [ ] Add graph/chart if data shows interesting patterns
- [ ] Document maximum reliable range achieved
- [ ] Note any limitations or recommendations

### NOTES.md Update
- [ ] Document sensor fusion learnings
  - BME680 vs SHT31-D comparison
  - When to use which sensor
  - Sensor fusion strategies
- [ ] Add insights about multi-sensor systems
- [ ] Note any calibration observations

### Final Git Commit
- [ ] Ensure all code changes committed
- [ ] Range testing results documented
- [ ] README and NOTES updated
- [ ] MILESTONE_LORA_COMMUNICATION.md finalized
- [ ] Commit message: "Week 2 complete - LoRa communication with range testing"
- [ ] Push to GitHub

---

## Success Criteria

**Week 2 Complete When:**
- [x] Multi-sensor integration working
- [x] LoRa communication end-to-end verified
- [x] Buffer size issues resolved
- [x] UART timing patterns documented
- [x] MILESTONE document created
- [ ] Range testing completed with data
- [ ] All documentation updated
- [ ] Code committed and pushed

**Phase 1 Progress**: Week 2 of 4 complete (50%)

---

## Notes

**Power Requirements**:
- Node 1 can run on battery (needs only MCU + sensors + LoRa)
- Node 2 needs USB connection for probe-rs monitoring (or use standalone mode with battery and check logs later)

**Alternative Approach** (if portable monitoring not feasible):
- Flash both nodes with standalone firmware
- Node 1: Transmit with LED indicators
- Node 2: Store statistics in memory, retrieve via USB after test
- Less real-time visibility but more portable

**Weather Considerations**:
- LoRa works better in clear weather
- Rain/fog can attenuate signal
- Document conditions for accurate results

---

*Last Updated*: December 27, 2024
*Priority*: Low (core functionality complete)
*Estimated Time*: 2-4 hours (including travel and testing)
