# Laser Scanner Parameters for Asphalt Crack Detection

## Optimal Configuration for Dark Asphalt Surfaces

### 1. Working Distance (Height from Ground)
**Recommended: 200-250mm**

```
                 Laser Scanner
                      |
                      | 200-250mm (OPTIMAL for asphalt)
                      |
                      v
              ================
                  Asphalt
```

**Why this distance:**
- ✓ Closer = More laser power on dark surface
- ✓ Better resolution for detecting small cracks (>0.1mm)
- ✓ Stronger signal return from low-reflectivity asphalt
- ✗ Too close (<150mm) = Out of focus, field of view too narrow
- ✗ Too far (>300mm) = Laser too weak on dark asphalt

**Test Range:** 180mm - 280mm (find sweet spot for your specific asphalt)

---

### 2. Scanner Settings (Exposure & Gain for Dark Surfaces)

#### Current Settings in Code:
```cpp
KSJ3D_SetExposureTime(0, 0.5);   // 0.5ms
KSJ3D_SetGain(0, 10);             // Gain = 10
```

#### **Recommended for Asphalt:**
```cpp
KSJ3D_SetExposureTime(0, 1.0);   // Increase to 1.0ms (darker surfaces need more exposure)
KSJ3D_SetGain(0, 15);             // Increase to 15 (boost signal on dark asphalt)
KSJ3D_Set3DLaserLineBrightnessThreshold(0, 8);  // Lower threshold (more lenient)
KSJ3D_Set3DLaserLineBrightnessLowThreshold(0, 255);
KSJ3D_Set3DLaserLineWidth(0, 25);  // Accept thinner lines
```

**Why these values:**
- **Exposure 1.0ms**: Asphalt absorbs light, needs longer exposure
- **Gain 15**: Amplifies weak reflections from dark surface
- **Threshold 8**: Lower = accepts dimmer laser reflections
- **Line Width 25**: More flexible for varying laser line quality

---

### 3. Movement Speed

**Recommended: 15-25 mm/s (1.5-2.5 cm/s)**

**Why slower for asphalt:**
- Crack detection requires HIGH resolution
- Darker surface = need more time per scan
- Typical asphalt cracks: 0.1mm - 5mm wide
- Slower speed = more profiles captured = better crack detection

**Resolution Calculation:**
```
Scan Rate: 100 Hz (100 profiles/second)
Robot Speed: 20 mm/s
Y-Resolution: 20mm/s ÷ 100Hz = 0.2mm between profiles

Result: Can detect cracks ≥ 0.2mm wide
```

**Speed vs Quality Trade-off:**
- **15 mm/s**: 0.15mm resolution - Best for fine cracks
- **20 mm/s**: 0.20mm resolution - Good balance ✓ **RECOMMENDED**
- **30 mm/s**: 0.30mm resolution - Faster, miss small cracks
- **50 mm/s**: 0.50mm resolution - Too fast for crack detection

---

### 4. Scan Pattern & Coverage

#### Single Scan Line Coverage
```
Field of View Width: ~100-150mm (depends on working distance)
Recommended Scan Line Spacing: 80-100mm
Overlap: 30-50% overlap between adjacent lines
```

#### Scan Pattern for Full Coverage:
```
Pass 1:  ═══════════════════════════
         (100mm)

Pass 2:      ═══════════════════════════
             (80mm offset - 20% overlap)

Pass 3:          ═══════════════════════════
                 (80mm offset)
```

**Overlap Benefits:**
- ✓ No missed areas
- ✓ Better crack continuity
- ✓ Cross-validation of detected cracks

---

### 5. ROI (Region of Interest)

**Use Maximum ROI:**
```cpp
int nColMax, nRowMax;
KSJ3D_GetRoiMax(0, &nColMax, &nRowMax);
KSJ3D_SetRoi(0, 0, 0, nColMax, nRowMax);  // Full sensor area
```

**Why maximum:**
- ✓ Capture entire laser line width
- ✓ Maximum coverage per scan
- ✓ Don't miss cracks at edges

**Typical Values:**
- Width (nColMax): ~2048-4096 pixels
- Height (nRowMax): ~1000-2000 profiles

---

### 6. Contrast Enhancement

For asphalt (low contrast surface), the laser line detection is critical:

```cpp
// Brightness thresholds - CRITICAL for dark asphalt
KSJ3D_Set3DLaserLineBrightnessThreshold(0, 8);      // Lower = more sensitive
KSJ3D_Set3DLaserLineBrightnessLowThreshold(0, 255); // Keep at max
KSJ3D_Set3DLaserLineWidth(0, 25);                   // Accept thinner lines
```

**Testing Strategy:**
1. Start with threshold = 10
2. If too many NaN points (>50%), lower to 8
3. If too noisy, increase to 12
4. Find balance for your specific asphalt type

---

### 7. Sample Size (Profiles per Scan)

**Current Setting:**
```cpp
KSJ3D_SetMaxNumberOfProfilesToCapture(0, 1000);  // 1000 profiles
```

**For Stationary Scanning (like demo):** Keep at 1000
**For Moving Robot:** Not applicable (continuous capture)

**At 20 mm/s and 100 Hz:**
- 1000 profiles = 10 seconds of scanning
- 10 seconds × 20 mm/s = 200mm scan length
- Perfect for testing!

---

### 8. Environmental Considerations

#### Lighting Conditions:
- **Best:** Overcast/diffuse lighting (no direct sun)
- **Avoid:** Direct sunlight (interferes with laser)
- **Indoor:** Fluorescent lights OK, but turn off if possible during scan

#### Temperature:
- Asphalt expands/contracts with temperature
- Best: 15-25°C
- Avoid: Very hot asphalt (>40°C) or freezing (<0°C)

#### Surface Preparation:
- Clean surface (no debris, water, oil)
- Dry asphalt only
- Wet asphalt = poor laser reflection

---

## Complete Configuration Code

Update your `laser_driver.cpp` with these optimized settings:

```cpp
void initializeScanner()
{
    // ... existing code ...

    // OPTIMIZED FOR ASPHALT CRACK DETECTION
    KSJ3D_SetExposureTime(0, 1.0);                          // Increased for dark surface
    KSJ3D_SetGain(0, 15);                                   // Boosted for low reflectivity
    KSJ3D_Set3DLaserLineBrightnessThreshold(0, 8);         // More sensitive
    KSJ3D_Set3DLaserLineBrightnessLowThreshold(0, 255);    // Maximum range
    KSJ3D_Set3DLaserLineWidth(0, 25);                      // Accept thinner lines
    KSJ3D_SetStartTrigger(0, STS_INPUT_0, 0, TEM_RISING_EDGE);
    KSJ3D_SetDataTriggerMode(0, DTM_INTERNAL);
    KSJ3D_SetDataTriggerInternalFrequency(0, 100);         // 100 Hz
    
    // Enable laser
    KSJ3D_LaserModeSet(0, LM_FLASH);
    
    KSJ3D_SetYResolution(0, 0.1);                          // 0.1mm Y resolution
    KSJ3D_SetMaxNumberOfProfilesToCapture(0, 1000);
    
    // ... rest of code ...
}
```

---

## Testing Protocol for Asphalt

### Step 1: Find Optimal Working Distance
```bash
# Test at different heights
Height: 180mm → Trigger scan → Check valid%
Height: 200mm → Trigger scan → Check valid%
Height: 220mm → Trigger scan → Check valid%
Height: 250mm → Trigger scan → Check valid%
Height: 280mm → Trigger scan → Check valid%

# Goal: Find height with highest valid% (aim for >80%)
```

### Step 2: Optimize Exposure/Gain
```bash
# If valid% is low (<50%):
- Increase exposure to 1.5ms
- Increase gain to 18
- Lower brightness threshold to 6

# If too noisy (lots of random points):
- Decrease exposure to 0.8ms
- Lower gain to 12
- Increase brightness threshold to 10
```

### Step 3: Test Movement Speed
```bash
# Place a ruler or known crack on asphalt
# Scan at different speeds:
15 mm/s → Check if crack is clearly visible
20 mm/s → Check if crack is clearly visible
25 mm/s → Check if crack is clearly visible
30 mm/s → Check if crack starts to blur

# Use slowest speed that captures crack clearly
```

### Step 4: Verify Resolution
```bash
# Place objects of known sizes on asphalt:
- 0.5mm wire
- 1.0mm crack
- 2.0mm crack

# Scan and verify all are detected in saved PCD
```

---

## Expected Results for Asphalt

### Good Scan Indicators:
- ✓ Valid points: **60-85%** (asphalt is dark, so some loss is normal)
- ✓ Z range variation: 0.5-3mm (typical asphalt surface roughness)
- ✓ Consistent point density
- ✓ Cracks clearly visible as depth changes
- ✓ Minimal noise around crack edges

### Problem Indicators:
- ✗ Valid points: <40% → Increase exposure/gain or move closer
- ✗ Valid points: <10% → Too far from surface or laser too weak
- ✗ All points at same Z → Surface too smooth or out of focus
- ✗ Random scattered points → Too much gain, decrease threshold

---

## Quick Reference Table

| Parameter | Concrete | Asphalt (Dark) | Metal/Bright |
|-----------|----------|----------------|--------------|
| Distance | 250-350mm | 200-250mm | 300-400mm |
| Exposure | 0.5ms | 1.0ms | 0.3ms |
| Gain | 10 | 15 | 5 |
| Threshold | 10 | 8 | 15 |
| Speed | 30 mm/s | 20 mm/s | 40 mm/s |
| Expected Valid% | 80-95% | 60-85% | 85-98% |

---

## Crack Detection Specifications

### Minimum Detectable Crack:
```
Working Distance: 220mm
Y-Resolution: 0.2mm (at 20 mm/s)
X-Resolution: ~0.1mm (across laser line)
Z-Resolution: 0.01-0.05mm (depth)

Minimum crack width: 0.2mm
Minimum crack depth: 0.1mm
Maximum scan width: ~120mm per pass
```

### Coverage Rate:
```
Speed: 20 mm/s
Scan width: 100mm
Coverage: 20mm/s × 100mm = 2,000 mm²/s = 7.2 m²/hour

For 1km road (3m wide):
Time = 3000m² ÷ 7.2m²/h = 417 hours (needs faster speed or wider coverage)
```

---

## Troubleshooting Asphalt Scans

### Problem: Very Few Valid Points
**Solution:**
1. Move scanner closer (try 200mm)
2. Increase exposure to 1.5ms
3. Increase gain to 18
4. Clean asphalt surface

### Problem: Noisy/Random Points
**Solution:**
1. Decrease gain to 12
2. Increase brightness threshold to 10
3. Check for ambient light interference
4. Slow down robot movement

### Problem: Can't See Small Cracks
**Solution:**
1. Reduce speed to 15 mm/s
2. Move closer to surface
3. Ensure proper focus
4. Check Z-resolution in saved PCD

### Problem: Cracks Appear Disconnected
**Solution:**
1. Increase scan line overlap to 50%
2. Reduce speed for better profile density
3. Check robot movement stability

---

## Summary - Recommended Starting Point

```
Working Distance: 220mm
Exposure: 1.0ms
Gain: 15
Brightness Threshold: 8
Line Width: 25 pixels
Robot Speed: 20 mm/s
Scan Rate: 100 Hz
ROI: Maximum
Scan Line Spacing: 80mm
Overlap: 20-30%
```

**Start here and adjust based on your specific asphalt type and lighting conditions!**
