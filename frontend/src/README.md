# EdgeSim Frontend

Frontend application for EdgeSim, a PyBullet-based warehouse simulation environment for mobile robotics data generation.

## Features

- **Text Mode**: Natural language prompts to describe warehouse scenarios
- **Visual Mode**: Graphical top-down 20m x 20m warehouse canvas for object placement
- **Comprehensive Configuration**: Full EdgeSim CLI parameter support
- **Dual Execution Modes**:
  - Test Run: Single simulation with GUI
  - Batch Data Creation: Multiple headless runs

## Prerequisites

- Node.js 18+ and npm/yarn/pnpm
- Anaconda environment with EdgeSim backend installed

## Installation

### 1. Install Node.js Dependencies

```bash
npm install
# or
yarn install
# or
pnpm install
```

### 2. Configure Backend Connection

Edit `vite.config.ts` to set your EdgeSim backend URL:

```typescript
server: {
  proxy: {
    '/api': {
      target: 'http://localhost:8000', // Your EdgeSim backend port
      changeOrigin: true,
    },
  },
}
```

## Development

Start the development server:

```bash
npm run dev
```

The application will be available at `http://localhost:3000`

## Building for Production

```bash
npm run build
```

The production build will be in the `dist` directory.

## Backend Integration

### Expected Backend Endpoints

The frontend expects the following API endpoints from your EdgeSim backend:

#### 1. Execute Simulation (POST /api/simulate)

```json
{
  "mode": "test" | "batch",
  "config": {
    // All EdgeSim CLI parameters
    "num_observations": 10,
    "headless": false,
    "scenario": {
      "type": "text" | "visual",
      "data": "..." // Text prompt or visual layout JSON
    }
    // ... other parameters
  }
}
```

Response:
```json
{
  "success": true,
  "job_id": "uuid",
  "message": "Simulation started"
}
```

#### 2. Get Simulation Status (GET /api/simulate/{job_id})

Response:
```json
{
  "status": "running" | "completed" | "failed",
  "progress": 0.5,
  "output_path": "/path/to/output"
}
```

### Example Backend Integration (Python/FastAPI)

Create a simple FastAPI backend in your Anaconda environment:

```python
# backend/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import subprocess
import uuid

app = FastAPI()

# Enable CORS for frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.post("/api/simulate")
async def run_simulation(config: dict):
    job_id = str(uuid.uuid4())
    
    # Build EdgeSim CLI command from config
    cmd = ["python", "-m", "edgesim"]
    
    if config.get("mode") == "batch":
        cmd.append("--headless")
    
    cmd.extend(["--num-observations", str(config.get("num_observations", 10))])
    
    # Add other CLI parameters from config
    # ...
    
    # Execute EdgeSim
    subprocess.Popen(cmd)
    
    return {
        "success": True,
        "job_id": job_id,
        "message": "Simulation started"
    }

@app.get("/api/simulate/{job_id}")
async def get_status(job_id: str):
    # Check job status
    return {
        "status": "running",
        "progress": 0.5
    }
```

Run the backend:

```bash
# In your Anaconda environment
conda activate your_env
pip install fastapi uvicorn
uvicorn backend.main:app --reload --port 8000
```

## Project Structure

```
edgesim-frontend/
├── components/
│   ├── ui/                 # Reusable UI components
│   ├── WarehouseObjects.tsx # Object definitions
│   └── figma/
│       └── ImageWithFallback.tsx
├── styles/
│   └── globals.css         # Global styles with Inter font
├── App.tsx                 # Main application component
├── main.tsx               # Application entry point
├── index.html             # HTML template
├── package.json           # Dependencies
├── vite.config.ts         # Vite configuration
└── tsconfig.json          # TypeScript configuration
```

## CLI Command Generation

The frontend generates EdgeSim CLI commands based on user configuration. Example output:

### Test Run:
```bash
python -m edgesim --num-observations 10 --scenario "warehouse with 3 racks and 2 robots"
```

### Batch Run:
```bash
python -m edgesim --headless --num-observations 100 --batch-size 10 --output-dir ./data
```

## Working with Anaconda

If you're using Anaconda for your EdgeSim backend:

1. Create a dedicated environment:
```bash
conda create -n edgesim python=3.10
conda activate edgesim
```

2. Install EdgeSim and dependencies:
```bash
pip install pybullet fastapi uvicorn
# Install your EdgeSim package
```

3. Run backend and frontend in separate terminals:

Terminal 1 (Backend):
```bash
conda activate edgesim
cd backend
uvicorn main:app --reload --port 8000
```

Terminal 2 (Frontend):
```bash
npm run dev
```

## Troubleshooting

### CORS Issues
If you see CORS errors, ensure your backend has CORS middleware configured (see backend example above).

### Port Conflicts
- Frontend default: `http://localhost:3000`
- Backend default: `http://localhost:8000`
- Change ports in `vite.config.ts` if needed

### Module Import Errors
Ensure all dependencies are installed:
```bash
npm install
```

## License

Proprietary - EdgeSim Warehouse Robotics Simulation
