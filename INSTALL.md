# Installation Guide

## System Requirements

- **Python**: 3.10 or later
- **OS**: Linux, macOS, or Windows (with a proper terminal emulator)
- **Terminal**: ≥ 80 × 24 characters (120 × 40 recommended for full cockpit layout)
- **Dependencies**: numpy, pytest (see `requirements.txt`)

## Installation Steps

### 1. Clone the repository

```bash
git clone https://github.com/themathmug/b737-flight-sim.git
cd b737-flight-sim
```

### 2. (Optional) Create a virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate        # Linux / macOS
.venv\Scripts\activate.bat       # Windows cmd
.venv\Scripts\Activate.ps1       # Windows PowerShell
```

### 3. Install dependencies

```bash
pip3 install -r requirements.txt
```

## Starting the Simulator

```bash
python3 main.py                     # default – idle on runway
python3 main.py --scenario 1        # takeoff scenario
python3 main.py --scenario 2        # cruise scenario
python3 main.py --list-scenarios    # list all available scenarios
python3 main.py --headless 60       # run 60 sim-seconds headlessly (no UI)
```

## Running Tests

```bash
python3 -m pytest tests/ -v
```

## Terminal Size Tips

- **macOS/Linux**: `resize -s 40 120` or drag terminal window larger
- **Windows Terminal**: Settings → Defaults → Initial size = 120 × 40
- If the display says "Terminal too small", resize and the display will update automatically.

## Troubleshooting

| Problem | Solution |
|---|---|
| `ModuleNotFoundError: numpy` | Run `pip3 install -r requirements.txt` |
| Blank/garbled display | Ensure terminal supports ANSI colour; try `export TERM=xterm-256color` |
| Curses error on Windows | Use Windows Terminal (≥ 1.0) or WSL |
| All instruments show 0 | Normal – apply throttle (`X` key) to start moving |
