# System Architecture Documentation

## Table of Contents
1. [System Overview](#system-overview)
2. [Pipelining Architecture](#pipelining-architecture)
3. [Camera Initialization](#camera-initialization)
4. [Camera Data Capture](#camera-data-capture)
5. [Frame Buffer](#frame-buffer)
6. [Preprocessing Pipeline](#preprocessing-pipeline)
7. [Processing Algorithms](#processing-algorithms)
8. [Postprocessing and Display](#postprocessing-and-display)
9. [Control System](#control-system)
10. [Motor Control](#motor-control)
11. [Special Features](#special-features)
12. [Clock Domains and Synchronization](#clock-domains-and-synchronization)

---

## System Overview

This is a real-time image processing system implemented on an FPGA that captures video from an OV7670 camera, processes it through various algorithms, and displays the results via HDMI. The system supports multiple processing modes including grayscale conversion, filtering, edge detection, skin detection, object tracking, gesture recognition, and even a two-player Pong game.

### High-Level Data Flow

```
Camera (OV7670) 
  → Camera Interface (pclk domain)
    → Frame Buffer (BRAM, dual-port)
      → Preprocessor (pclk domain)
        → Frame Buffer (write processed pixels)
          → VGA Controller (clk_25m domain)
            → Postprocessor (clk_25m domain)
              → HDMI Output
```

### Key Components

- **Camera Controller**: Initializes and captures data from OV7670
- **Frame Buffer**: Dual-port BRAM bridging camera and VGA clock domains
- **Preprocessor**: Real-time image processing pipeline
- **Postprocessor**: Channel selection, overlays, and final rendering
- **Control Unit**: State machine managing all processing modes
- **Motor Controller**: Pan/tilt servo control for object tracking
- **VGA Controller**: Generates 640x480@60Hz timing signals

---

## Pipelining Architecture

### What is Pipelining?

Pipelining is a fundamental architectural technique that breaks down complex operations into multiple sequential stages. Each stage processes data independently, allowing multiple data items to be processed simultaneously at different stages. This dramatically increases throughput by enabling parallel processing of different pixels or frames at the same time.

**Key Benefits:**
- **Increased Throughput**: While one pixel is being processed at a later stage, another pixel can be processed at an earlier stage
- **Resource Utilization**: Multiple hardware units can work simultaneously instead of sitting idle
- **Real-Time Performance**: Critical for video processing where each frame must be processed within 33ms (30fps)

**Pipeline Characteristics:**
- Each stage has a fixed latency (typically 1 clock cycle per stage)
- Data flows sequentially through stages
- Stages can operate in parallel on different data items
- The pipeline depth determines the initial latency (time until first output)

### System Pipeline Overview

This design implements a **multi-stage pipeline** with two main processing pipelines:

1. **Preprocessing Pipeline** (operates in pclk domain, ~24MHz)
2. **Postprocessing Pipeline** (operates in clk_25m domain, 25MHz)

The frame buffer acts as a pipeline register between these two domains, allowing them to operate independently.

### Preprocessing Pipeline (11 Stages)

The preprocessing pipeline processes pixels as they arrive from the camera. The exact number of active stages depends on the processing mode, but the maximum pipeline depth is **11 stages**:

1. **Grayscale Conversion** - Converts RGB444 to grayscale using luminance formula
2. **Line Buffering** - Buffers 3 lines to provide 3×3 pixel neighborhood
3. **Spatial Filtering** - Applies median filter or convolution kernel (Gaussian, Sobel, Sharpen, Emboss)
4. **Thresholding** - Binary threshold operation on filtered pixels
5. **Skin/Color Detection** - Parallel paths for skin threshold or color threshold
6. **Spatial Filter on Masks** - Noise reduction on binary masks (majority vote)
7. **Erosion** - Morphological operation to shrink blobs
8. **Dilation** - Morphological operation to expand blobs (after erosion)
9. **Blob Filtering** - Restricts processing to region around detected centroid
10. **Centroid Detection** - Calculates center of mass for detected objects
11. **Gesture Recognition** - Perceptron-based classifier for gesture detection

**Pipeline Flow Example (Skin Tracking Mode):**
```
Camera Pixel → Grayscale → Line Buffer → [Spatial Filter] → 
Skin Threshold → Line Buffer → Spatial Filter → 
Line Buffer → Erosion → Line Buffer → Dilation → 
Blob Filter → Centroid Detection → Frame Buffer
```

**Parallel Processing Paths:**
- The design supports multiple parallel processing paths:
  - **Grayscale path**: For spatial filtering operations
  - **Skin detection path**: Skin threshold → spatial filter → erosion → dilation
  - **Color detection path**: Color threshold → spatial filter
- These paths can operate simultaneously, with the output mux selecting the appropriate result

**Stage Latency:**
- Most stages: 1 clock cycle
- Line buffers: 1-2 clock cycles (coordinate lag)
- Centroid detection: Frame-based (outputs at end of frame)
- Total pipeline latency: ~10-15 clock cycles for typical processing modes

### Postprocessing Pipeline (5 Stages)

The postprocessing pipeline operates in the VGA clock domain and processes pixels as they are read from the frame buffer for display:

1. **Temporal Filter/Grayscale Replication** - Extracts temporal filter data or replicates grayscale across RGB channels
2. **Channel Selector** - Selectively displays RGB channels based on user control
3. **Text Overlay** - Renders state names, gesture text, and Pong scores
4. **Overlay Manager** - Draws crosshairs and bounding boxes
5. **Pong Game** - Renders game elements (paddles, ball, scores) when in Pong mode

**Pipeline Flow:**
```
Frame Buffer → Temporal/Grayscale → Channel Selector → 
Text Overlay → Overlay Manager → Pong Game → HDMI Output
```

**Stage Latency:**
- Each stage: 1 clock cycle
- Total pipeline latency: 5 clock cycles

### Overall System Pipeline

The complete system pipeline spans both clock domains:

```
Camera (pclk) → Preprocessor (11 stages, pclk) → 
Frame Buffer (pipeline register) → 
Postprocessor (5 stages, clk_25m) → HDMI (clk_25m)
```

**Total Pipeline Depth: 16 stages** (11 preprocessing + 1 frame buffer + 5 postprocessing)

**Key Design Decisions:**
- **Frame buffer as pipeline register**: Allows preprocessing and postprocessing to operate at different rates and independently
- **Conditional stage activation**: Not all stages are active in every mode, reducing power consumption
- **Parallel paths**: Multiple processing paths allow different algorithms to run simultaneously
- **Clock domain crossing**: Proper synchronization ensures data integrity between pipeline stages in different clock domains

**Performance Impact:**
- Pipeline enables real-time processing at 30fps (33ms per frame)
- Throughput: Can process one pixel per clock cycle at each stage
- Initial latency: ~15-20 clock cycles before first processed pixel appears (acceptable for video processing)

---

## Camera Initialization

### Overview

The camera initialization process configures the OV7670 camera module via the SCCB (Serial Camera Control Bus) protocol to output RGB444 format at 640x480 resolution.

### Initialization Sequence

1. **System Reset**: When the FPGA comes out of reset, `cam_start` is asserted
2. **Camera Initializer**: Reads configuration data from ROM and writes to camera registers
3. **SCCB Protocol**: Uses 400kHz I2C-like protocol to communicate with camera
4. **Completion**: `cam_done` signal indicates initialization complete

### Camera Initializer Module (`cam_initializer.sv`)

**State Machine**:
- `IDLE`: Waits for `cam_init_start` signal
- `SEND`: Reads ROM data and initiates SCCB write
- `TIMER`: Handles delays between commands
- `DONE`: Initialization complete

**ROM Data Format**:
- Each entry: 16 bits `{register_address[7:0], register_value[7:0]}`
- Special codes:
  - `0xFFFF`: End of configuration
  - `0xFFF0`: 10ms delay (allows camera to stabilize)

**Timing**:
- 10ms delay calculated as `(CLK_F * 10) / 1000` clock cycles
- Inter-command delay: 1 clock cycle minimum

### SCCB Master Module (`sccb_master.sv`)

**Protocol Details**:
- Clock: 400kHz (configurable via `SIOC_HZ` parameter)
- Device Address: `0x21` (7-bit, write) / `0x43` (7-bit, read)
- 3-phase write: Device ID + Register Address + Data
- 2-phase read: Device ID + Register Address, then repeated START + Device ID (read) + Data

**State Machine**:
- `IDLE`: Ready for new transaction
- `START_1/START_2`: Generate START condition (SDA high→low while SCL high)
- `WRITE_DEV`: Write device address (R/W=0 for write)
- `WRITE_REG`: Write register address
- `WRITE_DATA`: Write data byte
- `RESTART_1/RESTART_2`: Repeated START for read operations
- `WRITE_DEV_RD`: Write device address (R/W=1 for read)
- `READ_DATA`: Read data byte
- `ACK`: Acknowledge bit handling
- `STOP_1/STOP_2`: Generate STOP condition (SDA low→high while SCL high)
- `DONE`: Transaction complete

**Clock Generation**:
- SIOC clock derived from system clock
- Half-period counter: `SIOC_HALF_PERIOD = SYS_CLK_HZ / (2 * SIOC_HZ)`
- SIOC toggles when counter reaches half-period

**Data Handling**:
- SIOD is open-drain: drive low (0) or release to high-Z (1)
- Master drives data on SCL falling edge
- Master samples data on SCL rising edge (for reads)
- ACK bit: master releases SDA, slave can pull low

### Camera ROM (`cam_rom.sv`)

Stores register configuration values that set:
- Output format: RGB444
- Resolution: 640x480 (QVGA)
- Frame rate: ~30fps
- Color matrix and gain settings
- Clock prescaler and timing parameters

---

## Camera Data Capture

### Overview

The camera interface module (`cam_interface.sv`) captures pixel data from the OV7670 camera and converts it from the camera's byte stream format into 12-bit RGB444 pixels for storage in the frame buffer.

### Camera Output Format

The OV7670 outputs pixels as two 8-bit bytes per pixel in RGB444 format:
- **Byte 1**: `{R[3:0], G[3:2], xx}` - Red channel and upper 2 bits of green
- **Byte 2**: `{G[1:0], B[3:0], xx}` - Lower 2 bits of green and blue channel

**Reconstructed 12-bit pixel**: `{R[3:0], G[3:0], B[3:0]}`

### Capture State Machine

**States**:
1. `WAIT_FOR_INIT`: Waits for camera initialization to complete, skips first two frames
2. `FRAME_IDLE`: Waits for frame start (VSYNC falling edge)
3. `CAPTURE_PIXELS`: Actively capturing pixel data during frame

**Frame Synchronization**:
- `VSYNC`: Vertical sync signal (active high during frame blanking)
- Frame start: Falling edge of VSYNC (transition from blanking to active frame)
- Frame end: Rising edge of VSYNC (transition from active frame to blanking)
- `HREF`: Horizontal reference (active high during valid line data)

### Pixel Assembly Process

1. **Byte Toggle**: Tracks whether we're receiving first or second byte of pixel pair
2. **First Byte**: Stores lower 4 bits (`pixel_data[3:0]`) as `first_nibble`
3. **Second Byte**: Combines with first byte: `{first_nibble, pixel_data}` → 12-bit RGB444
4. **Address Calculation**: Linear address = `y * 640 + x`
   - X counter: 0-639, increments per pixel
   - Y counter: 0-479, increments per line
5. **Write Enable**: Asserted when valid pixel within frame bounds (640x480)

### Line Tracking

- `line_y`: Current line index (0-479) in pclk domain
- `line_ready_pulse`: 1-cycle pulse on HREF falling edge (end of line)
- Used by frame buffer for tear-free reading (avoids reading line currently being written)

### Coordinate System

- Origin: Top-left corner (0,0)
- X-axis: Left to right (0-639)
- Y-axis: Top to bottom (0-479)
- Address: Linear addressing `addr = y * 640 + x`

---

## Frame Buffer

### Overview

The frame buffer (`buffer.sv`) is a dual-port Block RAM (BRAM) that bridges the camera pixel clock domain (pclk) and the VGA display clock domain (clk_25m). It stores processed or raw pixel data and provides tear-free reading for display.

### Memory Organization

**Storage Format**:
- **Normal Mode**: 9-bit word stores single 3-bit grayscale value in `[8:6]`, `[5:0] = 0`
- **Temporal Filter Mode**: 9-bit word stores 3 frames (3 bits each):
  - `[8:6]`: Frame N (oldest)
  - `[5:3]`: Frame N+1
  - `[2:0]`: Frame N+2 (newest)
- **Color Mode**: 9-bit RGB333 format: `{R[2:0], G[2:0], B[2:0]}`

**Address Space**:
- 640 × 480 = 307,200 pixels
- 19-bit address: `addr[18:0] = y * 640 + x`

### Dual-Port BRAM

**Port A (Write - pclk domain)**:
- Writes from camera or preprocessor
- Supports read-modify-write for temporal filtering
- Address: `bram_addr_a` (muxed between read and write addresses)

**Port B (Read - clk_25m domain)**:
- Reads for VGA display
- 1-cycle latency (registered address)
- Address: `prefetch_addr_q` (registered to align with latency)

### Write Path Logic

**Data Source Selection**:
1. **Color Mode** (`!processing_enable`): Pack 12-bit RGB444 → 9-bit RGB333
   - `{cam_pix_data[11:9], cam_pix_data[7:5], cam_pix_data[3:1]}`
2. **Processing Mode** (`processing_enable && !temporal_filter_enable`): Store 3-bit processed pixel
   - `{processed_pixel[2:0], 6'b000000}`
3. **Temporal Filter Mode** (`temporal_filter_enable`): Read-modify-write
   - Stage 1: Read existing 9-bit value
   - Stage 2: Pack new 3-bit value into appropriate chunk based on `frame_counter`
   - Write modified value back

**Temporal Filter Packing**:
- `frame_counter = 0`: Write to `[8:6]`, preserve `[5:0]`
- `frame_counter = 1`: Write to `[5:3]`, preserve `[8:6]` and `[2:0]`
- `frame_counter = 2`: Write to `[2:0]`, preserve `[8:3]`

### Frame Counter (Modulo 3)

**Purpose**: Tracks which 3-bit chunk in the 9-bit word corresponds to the current frame for temporal filtering.

**Operation**:
- Increments on camera frame start (VSYNC falling edge)
- Cycles: 0 → 1 → 2 → 0
- Synchronized to clk_25m domain for VGA reading
- Latched per VGA frame to prevent mid-scan changes

**Frame Chunk Selection for Reading**:
- VGA reads the **previous** completed frame to avoid showing the chunk currently being written
- Mapping: current=0 → read chunk 2, current=1 → read chunk 0, current=2 → read chunk 1

### Read Path Logic

**Address Generation**:
- `prefetch_addr = xy_to_addr(draw_x, draw_y)`
- Registered as `prefetch_addr_q` to align with BRAM 1-cycle latency

**Tear Guard**:
- Avoids reading the line currently being written by camera
- Synchronizes `cam_line_ready` and `cam_line_y` to clk_25m domain
- If reading line Y and Y == `safe_line_y` (last completed line), read line Y-1 instead
- Only applies to first 2 pixels of line to prevent left-edge wrap

**Border Clamping**:
- Blanks first 2 and last 2 rows/columns to prevent edge artifacts
- Condition: `(draw_x < 2) || (draw_x >= 638) || (draw_y < 2) || (draw_y >= 478)`

### Compare Mode

When `compare_mode` is enabled (sw[13]):
- **Left half** (x < 320): Display processed pixels
- **Right half** (x >= 320): Display raw camera grayscale (green channel top 3 bits)
- Allows side-by-side comparison of processed vs. raw image

### Ripple Transition Effect

**Purpose**: Visual transition effect when override mode is toggled (btn[0] pressed).

**Mechanism**:
- Expanding circle from screen center (320, 240)
- Only pixels inside circle radius are written
- Radius expands by `RIPPLE_INCREMENT` (50 pixels) per camera frame
- Maximum radius: `RIPPLE_MAX_RADIUS` (400 pixels)
- Uses distance squared comparison to avoid square root: `dx² + dy² < radius²`

**Trigger**:
- `transition_trigger` pulse from control unit (on override toggle)
- Synchronized from clk_25m to pclk domain
- Edge-detected to generate single pulse

---

## Preprocessing Pipeline

### Overview

The preprocessor (`preprocessor.sv`) performs all real-time image processing operations before pixels are written to the frame buffer. It operates in the camera pixel clock domain (pclk) and processes pixels as they arrive from the camera.

### Pipeline Stages

The preprocessing pipeline consists of multiple stages that can be enabled/disabled based on the current processing mode:

1. **Grayscale Conversion**
2. **Line Buffering** (for spatial operations)
3. **Spatial Filtering** (Median, Convolution)
4. **Thresholding**
5. **Skin/Color Detection**
6. **Morphological Operations** (Erosion, Dilation)
7. **Blob Filtering**
8. **Centroid Detection**
9. **Gesture Recognition**

### Stage 1: Grayscale Conversion

**Module**: `grayscale_converter.sv`

**Algorithm**:
- Luminance formula: `Y = 0.299*R + 0.587*G + 0.114*B`
- Fixed-point approximation: `Y = (77*R + 150*G + 29*B) / 256`
- Coefficients chosen for 8-bit precision without overflow

**Implementation**:
1. Extract RGB components from 12-bit RGB444 input
2. Expand each 4-bit channel to 8-bit by replication
3. Multiply by fixed-point coefficients
4. Sum weighted values
5. Right shift by 8 bits (divide by 256)
6. Extract top 4 bits for 4-bit grayscale output
7. Replicate across RGB channels: `{gray_4bit, gray_4bit, gray_4bit}`

**Output**: 12-bit RGB444 with grayscale value in all channels

### Stage 2: Line Buffer

**Module**: `line_buffer.sv`

**Purpose**: Provides 3×3 pixel neighborhood for spatial filtering operations.

**Storage**:
- Three line buffers: `row0` (oldest), `row1` (middle), `row2` (newest)
- Each buffer: 640 pixels × 4 bits = 2,560 bits
- Total: 7,680 bits (implemented as distributed RAM/LUTs)

**Operation**:
1. Write incoming pixel to `row2[x_pos]`
2. At end of each row (x_pos == 639), shift buffers:
   - `row0` ← `row1`
   - `row1` ← `row2`
   - `row2` ← new line
3. Read 3×3 neighborhood:
   - Top row: `row0[x_left], row0[x_center], row0[x_right]`
   - Middle row: `row1[x_left], row1[x_center], row1[x_right]`
   - Bottom row: `row2[x_left], row2[x_center], row2[x_right]`

**Coordinate Calculation**:
- Center pixel (p11) coordinates lag by 1 clock due to write latency
- `out_x = (x_pos >= 1) ? (x_pos - 1) : 0`
- `out_y = (row_count >= 3) ? (y_pos - 1) : 0`

**Neighborhood Validity**:
- `neighborhood_valid = (row_count >= 3)` - requires at least 3 rows buffered

**Border Handling**:
- For x_pos < 2: clamp read indices to prevent negative addresses
- For x_pos == 640: clamp to last valid pixels

### Stage 3: Spatial Filtering

#### Median Filter

**Module**: `median_filter.sv`

**Algorithm**: Finds median of 9 pixels in 3×3 neighborhood using sorting network.

**Purpose**: Removes salt-and-pepper noise while preserving edges.

**Implementation**:
- Sorting network with 9 stages of compare-swap operations
- Ensures `sorted[4]` is the median value
- Bypass mode: outputs center pixel `p11` when disabled

**Advantages**:
- Excellent edge preservation
- Effective noise reduction
- No blurring of sharp features

#### Convolution Kernel

**Module**: `convolution_kernel.sv`

**Supported Kernels**:

1. **Gaussian Blur** (`kernel_select = 0`):
   - Enhanced kernel: `[2 8 2; 8 32 8; 2 8 2] / 64`
   - Creates pronounced blur effect
   - Calculation: weighted sum with rounding, clamp to 4-bit

2. **Sobel Edge Detection** (`kernel_select = 1`):
   - Horizontal gradient: `Gx = [-1 0 1; -2 0 2; -1 0 1]`
   - Vertical gradient: `Gy = [-1 -2 -1; 0 0 0; 1 2 1]`
   - Magnitude: `|Gx| + |Gy|` (approximation, avoids square root)
   - Clamp to 4-bit range (0-15)

3. **Sharpen** (`kernel_select = 2`):
   - Kernel: `[0 -1 0; -1 5 -1; 0 -1 0]`
   - Enhances edges and details
   - Clamp negative values to 0, positive values to 15

4. **Emboss** (`kernel_select = 3`):
   - Kernel: `[-2 -1 0; -1 1 1; 0 1 2]`
   - Creates 3D relief effect
   - Add 8 (mid-gray) to center result, then clamp

### Stage 4: Thresholding

**Module**: `threshold.sv`

**Algorithm**: Binary threshold on grayscale values.

**Operation**:
- Input: 12-bit RGB (grayscale replicated)
- Extract green channel (4-bit): `threshold_in[7:4]`
- Compare with `threshold_value` (0-15)
- Output: `threshold_out = (pixel >= threshold) ? 0xFFF : 0x000`
- Convert back to 3-bit: `thresholded_pixel = threshold_out[7:4] >> 1`

**Usage**: Creates binary mask from grayscale image for further processing.

### Stage 5: Skin Detection

**Module**: `skin_threshold.sv`

**Algorithm**: RGB-based skin color detection using multiple conditions.

**Color Space Conversion**:
1. Convert RGB444 (0-15) → RGB333 (0-7): take top 3 bits
2. Compute luma: `Y = (R + 2*G + B) >> 2`
3. Compute chroma differences: `Cr = R - G`, `Cb = B - G`

**Detection Conditions**:
- `Y ∈ [Y_MIN, Y_MAX]` (tunable via potentiometer)
- `Cr ∈ [1, 4]`
- `Cb ∈ [-3, 0]`
- `R > G`
- `R - B >= 2`
- `G >= B`

**Output**: Binary mask (4'hF = skin, 4'h0 = non-skin)

**Tuning**:
- `y_min_tune`: Signed 4-bit offset for Y minimum (from potentiometer)
- `y_max_tune`: Signed 4-bit offset for Y maximum (from potentiometer)
- Final range clamped to 0-7 (3-bit domain)

### Stage 6: Color Thresholding

**Module**: `color_threshold.sv`

**Algorithm**: Hamming distance-based color matching.

**Color Palette**: 16 predefined colors selected by `color_select[7:4]`:
- 0: Red, 1: Green, 2: Blue, 3: Yellow, 4: Magenta, 5: Cyan
- 6: White, 7: Black, 8: Orange, 9: Purple, A: Lime, B: Pink
- C: Teal, D: Olive, E: Gray, F: Brown

**Hamming Distance Calculation**:
1. XOR each channel with reference color
2. Count set bits (popcount) in each XOR result
3. Sum: `total_hamming = r_hamming + g_hamming + b_hamming`

**Matching**:
- Pixel matches if `total_hamming <= threshold`
- Output: `4'hF` if match, `4'h0` otherwise

**Threshold**: 4-bit value (0-15) from switches `sw[3:0]`

### Stage 7: Spatial Filter on Masks

**Module**: `spatial_filter.sv`

**Purpose**: Noise reduction on binary masks (skin or color).

**Algorithm**: 3×3 majority vote
- Count white pixels in neighborhood
- Output white if count >= threshold (typically 5/9)

**Implementation**: Uses same line buffer infrastructure as grayscale spatial filtering.

### Stage 8: Morphological Operations

**Module**: `morphological_filter.sv`

#### Erosion

**Algorithm**: Pixel is white only if ALL 8 neighbors are white.

**Effect**: Shrinks blobs, removes small noise specks.

**Implementation**:
- Check all 8 neighbors (excluding center)
- `eroded_pixel = center_pixel & (all_neighbors_white)`

#### Dilation

**Algorithm**: Pixel is white if ANY of 9 pixels (including center) is white.

**Effect**: Expands blobs, fills small holes.

**Implementation**:
- Check all 9 pixels in 3×3 window
- `dilated_pixel = any_pixel_white`

#### Opening (Erosion + Dilation)

**Effect**: Removes noise while restoring blob size.

**Usage**: Applied in sequence: erosion first, then dilation.

### Stage 9: Blob Filtering

**Module**: `blob_filter.sv`

**Purpose**: Restricts processing to region around previously detected centroid.

**Adaptive Padding**:
- Base padding: `PADDING_X = 40`, `PADDING_Y = 30`
- Motion padding: Additional `MOTION_PADDING_X = 25`, `MOTION_PADDING_Y = 20` when motion detected
- Motion detection: Centroid displacement > 15 pixels between frames

**Bounding Box Persistence**:
- Maintains last valid bbox for 3 frames after detection loss
- Prevents flickering when object temporarily leaves frame

**Edge Suppression**:
- Suppresses pixels within `EDGE_MARGIN = 4` pixels of frame borders
- Prevents bbox stretching when blob touches edges

**Operation**:
1. Calculate padded bbox: `[min_x - padding, min_y - padding, max_x + padding, max_y + padding]`
2. Clamp to frame bounds (0-639, 0-479)
3. Output mask pixel only if inside padded bbox and not on edge

### Stage 10: Centroid Detection

**Module**: `centroid_detector.sv`

**Algorithm**: Center of mass calculation using spatial moments.

**Moment Accumulation**:
- `M00 = sum(1)` for all white pixels (blob area)
- `M10 = sum(x)` for all white pixels
- `M01 = sum(y)` for all white pixels

**Centroid Calculation**:
- `centroid_x = M10 / M00`
- `centroid_y = M01 / M00`

**Implementation**:
- Accumulators: 20-bit for M00, 29-bit for M10/M01 (handles 640×480 frame)
- Division performed at end of frame
- Centroid valid only if `blob_area >= MIN_BLOB_SIZE` (300 pixels)

**Bounding Box**:
- Tracks min/max X and Y coordinates of white pixels
- Updated during frame scan
- Output at end of frame

**Frame Synchronization**:
- Accumulators reset on frame start (VSYNC rising edge)
- Centroid computed and output at end of frame

### Split Centroid Detection

**Module**: `split_centroid_detector.sv`

**Purpose**: Detects two separate centroids (left and right halves of frame).

**Algorithm**:
- Same as single centroid, but processes left half (x < 320) and right half (x >= 320) separately
- Two sets of accumulators and outputs
- Used for two-player Pong game

### Stage 11: Gesture Recognition

**Module**: `gesture_perceptron.sv`

**Algorithm**: Adaptive perceptron-based classifier with online learning.

**Features** (5 features, normalized to 0-255):
1. **Compactness**: `blob_area / bbox_area` (Q8.8 format)
2. **Velocity**: `|centroid_x - prev_centroid_x|` (pixels per frame)
3. **Area**: Normalized blob area
4. **Aspect Ratio**: `bbox_width / bbox_height` (approximated)
5. **Oscillation**: Direction change count (for wave detection)

**Perceptron Structure**:
- Three perceptrons (one per gesture class):
  - Fist: High compactness, low velocity, medium area
  - Open Hand: Low compactness, low velocity, large area
  - Wave: Medium compactness, high velocity, high oscillation
- Each perceptron: `activation = bias + sum(weight[i] * feature[i])`
- Winner-take-all: Highest activation above threshold wins

**Online Learning**:
- **Learning Mode** (sw[12] = 1): Buttons become teaching controls
  - btn[0]: Teach as FIST
  - btn[1]: Teach as OPEN HAND
  - btn[2]: Teach as WAVE
- **Learning Rule**: `weight = weight + α * error * feature`
  - `error = 1` if should be class but isn't (increase weights)
  - `error = -1` if shouldn't be class but is (decrease weights)
  - Learning rate: `α = 2` (Q4.4 format = 0.125)
- Weights persist across frames until reset

**Hysteresis**:
- Gesture must be detected for 2 consecutive frames before output
- Prevents flickering

**Velocity Tracking**:
- Tracks centroid X position between frames
- Computes signed velocity: `velocity = centroid_x - prev_centroid_x`
- Detects direction changes for oscillation feature

### Output Selection Logic

The preprocessor selects which processed pixel to output based on enabled processing stages:

**Priority Order**:
1. Color threshold with spatial filter
2. Color threshold without spatial filter
3. Blob filter
4. Skin threshold (raw)
5. Dilation (with erosion + spatial)
6. Erosion (with spatial)
7. Spatial filter on skin mask
8. Spatial processing (median/convolution + threshold)
9. Grayscale (passthrough)

**Border Handling**:
- Border pixels (first/last 2 rows/columns) output black (3'b000) for spatial operations
- Prevents edge artifacts from incomplete neighborhoods

**Address Generation**:
- Uses line buffer coordinates when spatial operations active
- Falls back to camera coordinates for per-pixel operations
- Function: `xy_to_addr(x, y) = y * 640 + x`

---

## Processing Algorithms

### Grayscale Conversion

**Formula**: `Y = 0.299*R + 0.587*G + 0.114*B`

**Fixed-Point Implementation**:
- Coefficients scaled to integers: `Y = (77*R + 150*G + 29*B) / 256`
- Avoids floating-point arithmetic
- Preserves perceived brightness

### Median Filter

**Algorithm**: Sort 9 pixels, output 5th element (median).

**Sorting Network**: 9 stages of compare-swap operations optimized for finding median.

**Complexity**: O(1) per pixel (fixed 9 elements).

### Gaussian Blur

**Kernel**: `[2 8 2; 8 32 8; 2 8 2] / 64`

**Calculation**:
```
sum = 2*p00 + 8*p01 + 2*p02 +
      8*p10 + 32*p11 + 8*p12 +
      2*p20 + 8*p21 + 2*p22
output = (sum + 32) >> 6  // Divide by 64 with rounding
```

### Sobel Edge Detection

**Gradients**:
- `Gx = -p00 + p02 - 2*p10 + 2*p12 - p20 + p22`
- `Gy = -p00 - 2*p01 - p02 + p20 + 2*p21 + p22`

**Magnitude**: `|Gx| + |Gy|` (approximation, avoids sqrt)

### Skin Detection

**Color Space**: RGB-based (no full YCbCr conversion for efficiency).

**Conditions** (all must be true):
- Luma in range: `Y ∈ [Y_MIN, Y_MAX]`
- Chroma red: `Cr ∈ [1, 4]`
- Chroma blue: `Cb ∈ [-3, 0]`
- Red dominance: `R > G` and `R - B >= 2`
- Green >= Blue: `G >= B`

### Morphological Opening

**Definition**: Erosion followed by dilation.

**Effect**: Removes small noise while preserving main blob size.

**Implementation**: Two separate line buffers and morphological filter instances.

### Centroid Calculation

**Moments**:
- `M00 = Σ 1` (area)
- `M10 = Σ x` (weighted sum of x)
- `M01 = Σ y` (weighted sum of y)

**Centroid**:
- `cx = M10 / M00`
- `cy = M01 / M00`

**Bit Widths**:
- M00: 20 bits (max 307,200 for 640×480)
- M10: 29 bits (max 196,608,000)
- M01: 29 bits (max 147,456,000)

---

## Postprocessing and Display

### Overview

The postprocessor (`postprocessor.sv`) handles final rendering stages between the frame buffer and HDMI output. It operates in the VGA clock domain (clk_25m) and adds overlays, text, and special effects.

### Pipeline Stages

1. **Temporal Filter Visualization**
2. **Grayscale Replication**
3. **Channel Selection**
4. **Text Overlay**
5. **Graphical Overlays** (crosshair, bounding boxes)
6. **Pong Game Rendering**

### Stage 1: Temporal Filter Visualization

**Module**: `temporal_filter.sv`

**Purpose**: Visualizes motion by computing differences between frames.

**Algorithm**:
1. Extract 3 frames from 9-bit word based on `frame_chunk_counter`
2. Compute pairwise differences: `diff1 = |current - prev1|`, `diff2 = |prev1 - prev2|`
3. Apply deadzone: ignore differences ≤ 2 (camera noise)
4. Sum differences: `motion = diff1 + diff2`
5. Color mapping:
   - `motion == 0`: Deep blue (no motion)
   - `motion < 3`: Blue → Cyan (low motion)
   - `motion < 6`: Cyan → Green → Yellow (medium motion)
   - `motion >= 6`: Yellow → Orange → Red (high motion)

**Output**: RGB heatmap showing motion intensity.

### Stage 2: Grayscale Replication

**Operation**:
- In processing mode: Replicate 3-bit grayscale value from red channel `[8:6]` across all RGB channels
- In color mode: Use all channels normally
- In temporal filter mode: Use heatmap RGB from temporal filter

### Stage 3: Channel Selection

**Module**: `channel_selector.sv`

**Purpose**: Selectively display RGB channels based on `channel_select[2:0]`.

**Modes**:
- `111`: All channels (normal color)
- `100`: Red only
- `010`: Green only
- `001`: Blue only
- `110`: Red + Green (yellow)
- `101`: Red + Blue (magenta)
- `011`: Green + Blue (cyan)
- `000`: No channels (black)

**Bypass**: When `force_color` or `force_grayscale` is active, channel selection is bypassed.

### Stage 4: Text Overlay

**Module**: `text_overlay.sv`

**Features**:
- State name display at bottom of screen
- Gesture text (FIST, OPEN, WAVE) when gesture detected
- Pong score display (large numbers)
- Win status messages

**Font Rendering**:
- Uses `font_rom.sv` for character bitmaps
- 8×8 pixel characters
- Monochrome (white text on black background or vice versa)

**Gesture Latching**:
- Gesture code latched per frame to prevent flicker
- Priority: WAVE > FIST > OPEN
- Reset at frame start

### Stage 5: Graphical Overlays

**Module**: `overlay_manager.sv`

**Crosshair**:
- Drawn at centroid position when `overlay_enable` is active
- Length: 20 pixels, thickness: 2 pixels
- Center gap: 5 pixels
- Color: White or green

**Bounding Box**:
- Drawn when `bbox_overlay_enable` is active (sw[14])
- Rectangle outline around detected blob
- Color: Yellow or cyan

**Dual Centroid Mode**:
- When `split_centroid_enable` is active, draws two crosshairs
- Left centroid: Left side of screen
- Right centroid: Right side of screen
- Different colors for distinction

**Rendering Priority**:
1. Background pixels (from frame buffer)
2. Bounding box (if enabled)
3. Crosshair (if enabled)
4. Text overlay (always on top)

### Stage 6: Pong Game

**Module**: `pong.sv`

**Game Mechanics**:
- Two paddles controlled by left/right centroids
- Ball bounces off walls and paddles
- Scoring: Ball passes opponent's paddle
- Win condition: First to 7 points

**Ball Physics**:
- Constant speed: `BALL_SPEED_X = 3`, `BALL_SPEED_Y = 2` pixels per frame
- Collision detection:
  - Top/bottom walls: Reverse Y direction
  - Paddles: Reverse X direction
  - Paddle collision zone: Paddle center ± 40 pixels (half paddle height)

**Paddle Control**:
- Left paddle: Controlled by right centroid Y position (mirrored)
- Right paddle: Controlled by left centroid Y position (mirrored)
- Paddles follow centroid when valid, otherwise stay at center

**Visual Elements**:
- White paddles (5×80 pixels)
- Green ball (8×8 pixels)
- Red dots at centroid positions (6×6 pixels)
- Dashed center line (red, every 16 pixels)
- Score display (large numbers via text overlay)

**State Management**:
- Ball position and velocity updated on frame start
- Score updated when ball passes loss boundaries
- Rematch: btn[0] resets game (only in PONG state)

**Rendering**:
- Ball (highest priority)
- Red dots (centroid indicators)
- White paddles
- Gray center line
- Background (from frame buffer)

---

## Control System

### Overview

The control unit (`control_unit.sv`) is a finite state machine that manages all processing modes and user interactions. It provides enable signals to various processing stages based on the current state.

### State Machine

**States** (22 total):
0. `RAW`: Raw color output
1. `CHANNEL_MODE`: Individual channel display
2. `GRAYSCALE`: Grayscale converted output
3. `THRESHOLD`: Binary threshold on grayscale
4. `TEMPORAL`: Temporal filter (moving average)
5. `MEDIAN`: Median filter then threshold
6. `GAUSSIAN`: Gaussian blur
7. `SOBEL`: Sobel edge detection
8. `SHARPEN`: Sharpen filter
9. `EMBOSS`: Emboss 3D relief
10. `SKIN`: Skin thresholding
11. `SPATIAL`: Spatial filter on skin mask
12. `ERODE`: Erosion
13. `DILATE`: Dilation
14. `CENTROID`: Centroid detection
15. `BLOB`: Blob filtering
16. `SKIN_MOTION`: Skin motion tracking (motors enabled)
17. `COLOR_THRESH`: Color thresholding
18. `COLOR_TRACK`: Color tracking + motion
19. `GESTURE`: Gesture detection
20. `DUAL_CENT`: Dual centroid (left/right)
21. `PONG`: Pong game mode

### Navigation

**Buttons** (when `sw[12] = 0`):
- `btn[0]`: Toggle override mode (force color/grayscale)
- `btn[1]`: Next state
- `btn[2]`: Previous state

**Override Mode** (`btn[0]` pressed):
- `sw[15] = 1`: Force raw color display
- `sw[15] = 0`: Force plain grayscale display
- Processing still runs in background for overlays

**Learning Mode** (`sw[12] = 1`):
- Buttons become gesture teaching controls
- `btn[0]`: Teach as FIST
- `btn[1]`: Teach as OPEN HAND
- `btn[2]`: Teach as WAVE
- State navigation disabled

### Switch Controls

- `sw[2:0]`: Channel select (RGB) in CHANNEL_MODE state
- `sw[3:0]`: Color threshold value (Hamming distance) in COLOR_THRESH/COLOR_TRACK
- `sw[12]`: Learning mode toggle
- `sw[13]`: Compare mode (split screen)
- `sw[14]`: Bounding box overlay enable
- `sw[15]`: Color/grayscale override with btn[0]

### Potentiometer Controls

- `pot[15:12]`: Threshold value (0-15) for THRESHOLD, MEDIAN, GAUSSIAN, SOBEL, SHARPEN, EMBOSS
- `pot[15:12]`: Skin Y min (high-res) for skin detection states
- `pot[11:8]`: Skin Y max (low-res) for skin detection states
- `pot[15:8]`: Color selection (8-bit) for COLOR_THRESH/COLOR_TRACK

### Output Signals

The control unit generates enable signals for:
- `grayscale_enable`
- `threshold_enable` + `threshold_value`
- `median_enable`
- `convolution_enable` + `kernel_select`
- `skin_threshold_enable` + `skin_y_min` + `skin_y_max`
- `spatial_filter_enable`
- `erosion_enable`
- `dilation_enable`
- `centroid_enable`
- `blob_filter_enable`
- `motion_enable`
- `color_threshold_enable` + `color_select` + `color_threshold_value`
- `temporal_filter_enable`
- `gesture_enable`
- `split_centroid_enable`
- `force_color` / `force_grayscale`
- `channel_mode_enable` + `channel_select`
- `transition_trigger` (for ripple effect)

### State Transitions

**Next State Logic**:
- `btn[1]` pressed: Cycle forward through states
- `btn[2]` pressed: Cycle backward through states
- Wraps around: PONG → RAW, RAW → PONG

**Override Handling**:
- When override active, processing still runs but display is forced
- Ensures overlays (crosshair, text) still work correctly

---

## Motor Control

### Overview

The motor controller (`motor_controller.sv`) provides pan/tilt servo control for object tracking. It centers the detected object in the camera frame by commanding motor movements.

### Control Algorithm

**Deadzone**:
- Horizontal: ±60 pixels from center (320)
- Vertical: ±40 pixels from center (240)
- Prevents jitter when object is near center

**Step Size Calculation** (Adaptive):
- Horizontal:
  - 0-60px: Deadzone (no movement)
  - 61-100px: 100 steps/clock
  - 101-165px: 125 steps/clock
  - 166-200px: 150 steps/clock
  - 201-250px: 175 steps/clock
  - 251-320px: 200 steps/clock
- Vertical:
  - 0-40px: Deadzone
  - 41-100px: 100 steps/clock
  - 101-145px: 125 steps/clock
  - 146-190px: 150 steps/clock
  - 191-240px: 175 steps/clock

**Direction Calculation**:
- `offset_x = centroid_x - 320`
- `offset_y = centroid_y - 240`
- If `|offset_x| > DEADZONE_X`:
  - `offset_x > 0`: Move right (direction[0] = 1)
  - `offset_x < 0`: Move left (direction[3] = 1)
- If `|offset_y| > DEADZONE_Y`:
  - `offset_y > 0`: Move down (direction[2] = 1)
  - `offset_y < 0`: Move up (direction[1] = 1)

**Enable Condition**:
- Motors enabled only when `motion_enable && centroid_valid`
- Prevents runaway when no object detected

### Motor Driver

**Module**: `motor_driver.sv`

**Function**: Converts direction and step size into PWM signals for servos.

**PWM Generation**:
- Pan servo: `pwm_pan` signal
- Tilt servo: `pwm_tilt` signal
- Duty cycle varies based on step size and direction

**Position Tracking**:
- Maintains current pan/tilt positions
- Updates based on step size and direction
- Timeout mechanism returns to center when disabled

### Status Outputs

- `motor_direction[3:0]`: Direction bits (displayed on LEDs[15:12])
  - `[3]`: Left
  - `[2]`: Down
  - `[1]`: Up
  - `[0]`: Right
- `error_x`, `error_y`: Centroid offset from center (for debugging)

---

## Special Features

### Compare Mode

**Activation**: `sw[13] = 1`

**Operation**:
- Left half (x < 320): Display processed pixels
- Right half (x >= 320): Display raw camera grayscale

**Implementation**: Frame buffer writes processed pixels to left half, raw grayscale to right half.

### Ripple Transition Effect

**Trigger**: Override toggle (btn[0] pressed when not in learning mode)

**Effect**: Expanding circle from screen center reveals new image.

**Mechanism**:
- Only pixels inside expanding circle radius are written
- Radius increases by 50 pixels per camera frame
- Maximum radius: 400 pixels (covers entire 640×480 screen in ~8 frames)
- Uses distance squared comparison: `(x-320)² + (y-240)² < radius²`

### Temporal Filtering

**Purpose**: Reduce noise by averaging across multiple frames.

**Storage**: 9-bit word stores 3 frames (3 bits each).

**Frame Counter**: Tracks which chunk is current frame (0, 1, or 2).

**Read-Modify-Write**:
1. Read existing 9-bit value
2. Extract appropriate chunk based on `frame_counter`
3. Pack new 3-bit value into correct position
4. Write modified 9-bit value

**Visualization**: Motion heatmap showing frame-to-frame differences.

### Gesture Learning

**Activation**: `sw[12] = 1` (learning mode)

**Teaching**:
- Show gesture clearly
- Press corresponding button:
  - `btn[0]`: FIST
  - `btn[1]`: OPEN HAND
  - `btn[2]`: WAVE
- Release button before changing gesture
- Weights update on frame start when button pressed

**Learning Rule**: Perceptron update with learning rate α = 0.125

**Persistence**: Weights persist until hardware reset

### Pong Game

**Activation**: State 21 (PONG)

**Controls**:
- Left player: Right hand controls left paddle (mirrored)
- Right player: Left hand controls right paddle (mirrored)
- `btn[0]`: Rematch/reset game

**Scoring**: First to 7 points wins

**Visual Feedback**:
- Red dots show centroid positions
- White paddles
- Green ball
- Dashed center line
- Large score display

---

## Clock Domains and Synchronization

### Clock Domains

**Three Main Domains**:
1. **clk (100MHz)**: System clock, control unit, hex displays
2. **pclk (24MHz)**: Camera pixel clock, preprocessor
3. **clk_25m (25MHz)**: VGA/HDMI clock, postprocessor

**Additional Clocks**:
- **clk_125m (125MHz)**: HDMI serializer clock (5× pixel clock)
- **xclk (24MHz)**: Camera master clock

### Clock Generation

**Module**: `clk_wiz_0` (Xilinx Clock Wizard IP)

**Outputs**:
- `clk_out1`: 25MHz (VGA)
- `clk_out2`: 125MHz (HDMI serializer)
- `clk_out3`: 24MHz (camera xclk)

**Input**: 100MHz system clock

### Reset Synchronization

**Reset Distribution**:
1. External reset button → debouncer
2. Debounced reset synchronized to each clock domain:
   - `rst_sync_clk`: 2-flop synchronizer for clk domain
   - `rst_sync_25m`: 2-flop synchronizer for clk_25m domain
   - `rst_sync_pclk`: 2-flop synchronizer for pclk domain

**Synchronizer Module**: `sync_flop.sv` (2-flop synchronizer)

### Cross-Domain Signals

**Camera → System**:
- `cam_vsync`: Synchronized to pclk domain (already in pclk)
- `cam_line_ready`: Synchronized to clk_25m domain (2-flop)
- `cam_line_y`: Sampled when `cam_line_ready` synchronized (multi-bit CDC)

**Control → Preprocessor**:
- All control signals synchronized to pclk domain
- Processing enable signals

**Preprocessor → Buffer**:
- `processed_pixel`, `processed_addr`, `processed_valid`: All in pclk domain

**Buffer → Postprocessor**:
- `vga_pix_data`: Read in clk_25m domain
- `frame_chunk_counter`: Synchronized from pclk to clk_25m, latched per VGA frame

**Centroid → Motor Controller**:
- `centroid_x`, `centroid_y`, `centroid_valid`: Synchronized to clk_25m domain

**Control → Buffer**:
- `transition_trigger`: Synchronized from clk_25m to pclk domain (2-flop + edge detect)

### Synchronization Techniques

**2-Flop Synchronizer**:
- Used for single-bit signals
- Prevents metastability
- Adds 2 clock cycles of latency

**Multi-Bit Sampling**:
- For `cam_line_y`: Sample only when `cam_line_ready` pulse is synchronized
- Reduces CDC hazards by sampling when signal is stable

**Frame-Based Latching**:
- `frame_chunk_counter`: Latched at VGA frame start (draw_x=0, draw_y=0)
- Prevents mid-scan changes that cause visible artifacts

**Handshaking**:
- Not used (all signals are unidirectional)
- Relies on frame-based timing for coordination

### Tear-Free Reading

**Problem**: VGA reads frame buffer while camera writes to it.

**Solution**:
1. Track last completed line (`safe_line_y`) in clk_25m domain
2. If VGA reads line Y and Y == `safe_line_y`, read line Y-1 instead
3. Only applies to first 2 pixels of line (prevents left-edge wrap)
4. Border clamping blanks first/last 2 rows/columns

**Line Ready Pulse**:
- Generated on HREF falling edge (end of line) in pclk domain
- Synchronized to clk_25m domain
- Used to update `safe_line_y`

---

## Summary

This system implements a complete real-time image processing pipeline on an FPGA, from camera capture through various processing algorithms to HDMI display. Key achievements:

1. **Real-Time Processing**: All operations complete within one frame time (33ms at 30fps)
2. **Multiple Processing Modes**: 22 different states demonstrating various algorithms
3. **Object Tracking**: Centroid detection with motor control for pan/tilt tracking
4. **Gesture Recognition**: Adaptive perceptron-based classifier with online learning
5. **Interactive Game**: Two-player Pong controlled by hand tracking
6. **Robust Synchronization**: Proper clock domain crossing for reliable operation

The architecture is modular, with each component handling a specific function, making it maintainable and extensible.

