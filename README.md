# LX-Beats
Taking live heart-rate from 60 people at the same time using 20 NPG Lite devices connected to a single ESP32-S3 dongle to control a live animation on a web interface!

**Data Structure:**
- 20 nodes × 3 persons each = 60 simultaneous heart rate readings
- RMS calculation aggregates BPM from all connected nodes to a single BPM value
- Time-series data with event markers for performance analysis

## Setup

1. Place all files in the same directory
2. Serve via local web server: `python -m http.server 8000`
3. Open `http://localhost:8000/index.html`

## Features

- **BPM Plot**: Color-coded heart rate bars (Green <90, Orange 90-110, Red >110)
- **Nodes Plot**: Device connectivity (Red <5, Orange 5-10, Green >10 connected devices)
- **Event Markers**: Purple circles mark performance events - hover for full data
- **Performance Ranges**: Red background indicates active performance periods
- **Interactive Timeline**: Drag, zoom, and scroll through time-series data
- **Dual Data Sources**: Toggle between Raw and Processed data instantly

## Navigation & Controls

**Timeline Navigation:**
- **Drag Timeline Bar**: Click and drag the grey timeline bar to scroll through time
- **Mouse Wheel**: Scroll up/down to move forward/backward through data
- **Zoom Slider**: Drag left/right arrows on the slider to adjust visible time window
- **Slider Handle**: Drag the entire green slider handle to jump to different time periods

**Data Interpretation:**
- **BPM Bars**: Green (<90) = relaxed, Orange (90-110) = normal, Red (>110 or clamped) = elevated
- **Nodes Bars**: Red (<5) = poor connection, Orange (5-10) = moderate, Green (>10) = good connection
- **Purple Markers**: Event indicators (hover for details) - clickable circles at plot tops
- **Red Background**: Performance duration (from start to freeze frame)
- **Hover Any Bar**: Shows Time, BPM, Connected Nodes, and Confidence %

**Data Source Toggle:**
- Switch between "Raw Data" and "Processed Data" buttons at top-right
- Data is cached for instant switching

## Data Processing

The project includes a Jupyter notebook [`ISF-2026-Data-Processing.ipynb`](ISF-2026-Data-Processing.ipynb) for processing raw BPM data:

**Processing Steps:**
1. **Event Marker Integration**: Merges event markers from [`ISF-2026-Event-Markers.csv`](ISF-2026-Event-Markers.csv) into the main dataset
2. **Raw Data Export**: Generates [`ISF-2026-BPM-Data-Raw-with-events.csv`](ISF-2026-BPM-Data-Raw-with-events.csv) with all 20 nodes
3. **Data Filtering**: Creates processed dataset by:
   - Removing Node14 (disconnected device)
   - Filtering out rows with ConnectedNodes < 2
4. **Processed Data Export**: Generates [`ISF-2026-BPM-Data-Processed-with-events.csv`](ISF-2026-BPM-Data-Processed-with-events.csv)

**Requirements**: `pandas`, `pathlib`

**Usage**: Run all cells sequentially to regenerate both CSV files from source data.

## Files Required

- `ISF-2026-BPM-Data-Raw-with-events.csv`
- `ISF-2026-BPM-Data-Processed-with-events.csv`
- `index.html`

---

