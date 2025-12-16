# EdgeSim Frontend - Quick Setup Guide for Anaconda Users

## Step-by-Step Setup Instructions

### Step 1: Download Your Project Files

All your project files are ready in this Figma Make environment. You need to download them to your local machine:

**Essential Files to Download:**
1. `package.json` - Dependencies configuration
2. `vite.config.ts` - Vite bundler configuration
3. `tsconfig.json` - TypeScript configuration
4. `tsconfig.node.json` - TypeScript node configuration
5. `index.html` - HTML entry point
6. `main.tsx` - React entry point
7. `.gitignore` - Git ignore rules
8. `App.tsx` - Main application component
9. `styles/globals.css` - Global styles with Inter font
10. `components/WarehouseObjects.tsx` - Warehouse object definitions
11. All files in `components/ui/` - UI component library

### Step 2: Set Up Local Project Directory

```bash
# Create project directory
mkdir edgesim-frontend
cd edgesim-frontend

# Create directory structure
mkdir -p components/ui
mkdir -p components/figma
mkdir -p styles
```

### Step 3: Copy Files to Local Directory

Copy all the downloaded files into your `edgesim-frontend` directory, maintaining the folder structure:

```
edgesim-frontend/
├── package.json
├── vite.config.ts
├── tsconfig.json
├── tsconfig.node.json
├── index.html
├── main.tsx
├── App.tsx
├── .gitignore
├── styles/
│   └── globals.css
└── components/
    ├── WarehouseObjects.tsx
    ├── figma/
    │   └── ImageWithFallback.tsx
    └── ui/
        ├── accordion.tsx
        ├── alert-dialog.tsx
        ├── alert.tsx
        ├── badge.tsx
        ├── button.tsx
        ├── card.tsx
        ├── checkbox.tsx
        ├── dialog.tsx
        ├── input.tsx
        ├── label.tsx
        ├── popover.tsx
        ├── radio-group.tsx
        ├── scroll-area.tsx
        ├── select.tsx
        ├── separator.tsx
        ├── slider.tsx
        ├── switch.tsx
        ├── tabs.tsx
        ├── textarea.tsx
        ├── tooltip.tsx
        └── utils.ts
```

### Step 4: Install Node.js (if not already installed)

Download and install Node.js from https://nodejs.org/ (LTS version recommended)

Verify installation:
```bash
node --version  # Should show v18 or higher
npm --version
```

### Step 5: Install Project Dependencies

```bash
cd edgesim-frontend
npm install
```

This will install all required packages including React, Vite, Tailwind CSS, and all UI components.

### Step 6: Set Up Your EdgeSim Backend (Anaconda)

#### Option A: Simple Backend (Just CLI Command Generation)

If you only want the frontend to generate CLI commands for you to run manually:

1. The frontend is already configured to show the generated CLI command in a dialog
2. You can copy the command and run it directly in your Anaconda environment
3. **No backend server needed!**

#### Option B: Full Backend Integration (API Server)

If you want the frontend to automatically trigger EdgeSim simulations:

1. **Create a FastAPI backend in your Anaconda environment:**

```bash
# Activate your Anaconda environment
conda activate your_edgesim_env  # or create new: conda create -n edgesim python=3.10

# Install FastAPI and Uvicorn
pip install fastapi uvicorn python-multipart
```

2. **Create a simple backend file:**

Create `backend/main.py` in your EdgeSim project directory:

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import subprocess
import uuid
import os
from typing import Optional, Dict, Any

app = FastAPI(title="EdgeSim Backend API")

# Enable CORS for frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class SimulationConfig(BaseModel):
    mode: str  # "test" or "batch"
    config: Dict[str, Any]

# Store running jobs (in production, use a database)
jobs = {}

@app.post("/api/simulate")
async def run_simulation(request: SimulationConfig):
    """Execute EdgeSim simulation based on frontend config"""
    job_id = str(uuid.uuid4())
    config = request.config
    
    # Build EdgeSim CLI command
    cmd = ["python", "-m", "edgesim"]  # Adjust to your EdgeSim entry point
    
    # Add parameters from config
    if request.mode == "batch":
        cmd.append("--headless")
    
    if "num_observations" in config:
        cmd.extend(["--num-observations", str(config["num_observations"])])
    
    if "num_episodes" in config:
        cmd.extend(["--num-episodes", str(config["num_episodes"])])
    
    if "output_dir" in config:
        cmd.extend(["--output-dir", config["output_dir"]])
    
    # Add scenario (text or visual)
    if "scenario_type" in config:
        if config["scenario_type"] == "text" and "text_prompt" in config:
            cmd.extend(["--scenario", config["text_prompt"]])
        elif config["scenario_type"] == "visual" and "visual_layout" in config:
            # Save visual layout to temp file and pass path
            import json
            layout_file = f"/tmp/layout_{job_id}.json"
            with open(layout_file, 'w') as f:
                json.dump(config["visual_layout"], f)
            cmd.extend(["--scenario-file", layout_file])
    
    # Execute EdgeSim in background
    try:
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        jobs[job_id] = {
            "status": "running",
            "process": process,
            "command": " ".join(cmd)
        }
        
        return {
            "success": True,
            "job_id": job_id,
            "command": " ".join(cmd),
            "message": "Simulation started successfully"
        }
    except Exception as e:
        return {
            "success": False,
            "error": str(e)
        }

@app.get("/api/simulate/{job_id}")
async def get_simulation_status(job_id: str):
    """Check status of running simulation"""
    if job_id not in jobs:
        return {"error": "Job not found"}
    
    job = jobs[job_id]
    process = job["process"]
    
    # Check if process is still running
    if process.poll() is None:
        return {
            "status": "running",
            "job_id": job_id
        }
    else:
        return_code = process.returncode
        stdout, stderr = process.communicate()
        
        return {
            "status": "completed" if return_code == 0 else "failed",
            "job_id": job_id,
            "return_code": return_code,
            "output": stdout,
            "error": stderr
        }

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

3. **Run the backend:**

```bash
# In your Anaconda environment
conda activate your_edgesim_env
cd backend
python main.py
# or
uvicorn main:app --reload --port 8000
```

### Step 7: Configure Frontend to Connect to Backend

Edit `vite.config.ts` to set your backend URL (already configured for `http://localhost:8000`):

```typescript
server: {
  port: 3000,
  proxy: {
    '/api': {
      target: 'http://localhost:8000',  // Your backend URL
      changeOrigin: true,
      rewrite: (path) => path.replace(/^\/api/, ''),
    },
  },
}
```

### Step 8: Run the Frontend

```bash
cd edgesim-frontend
npm run dev
```

The application will open at `http://localhost:3000`

### Step 9: Test the Integration

1. Open `http://localhost:3000` in your browser
2. Choose Text Mode or Visual Mode
3. Configure your warehouse scenario
4. Click "Start Test Run" or "Create Batch Data"
5. The frontend will show the generated CLI command
6. If backend is running, it will automatically execute the simulation

## Workflow Options

### Workflow 1: Manual (No Backend Required)

1. Use the frontend to configure your simulation
2. Click execute button
3. Copy the generated CLI command from the dialog
4. Run it manually in your Anaconda terminal:
   ```bash
   conda activate your_edgesim_env
   python -m edgesim --num-observations 10 --scenario "your scenario"
   ```

### Workflow 2: Automated (With Backend)

1. Start backend: `conda activate edgesim && python backend/main.py`
2. Start frontend: `npm run dev`
3. Configure and execute simulations through the UI
4. Backend automatically runs EdgeSim and reports status

## Anaconda Environment Setup (Complete Example)

```bash
# Create new Anaconda environment
conda create -n edgesim python=3.10
conda activate edgesim

# Install PyBullet and dependencies
pip install pybullet numpy

# Install your EdgeSim package
cd /path/to/your/edgesim
pip install -e .

# Install backend dependencies (if using API)
pip install fastapi uvicorn python-multipart

# Test EdgeSim works
python -m edgesim --help
```

## Troubleshooting

### Issue: "npm: command not found"
**Solution:** Install Node.js from https://nodejs.org/

### Issue: "Cannot find module '@radix-ui/...'"
**Solution:** Run `npm install` again

### Issue: CORS errors in browser console
**Solution:** Make sure your backend has CORS middleware configured (see backend example above)

### Issue: Backend not accessible
**Solution:** 
- Check backend is running: `curl http://localhost:8000/health`
- Check Anaconda environment is activated
- Check firewall settings

### Issue: Port 3000 already in use
**Solution:** Change port in `vite.config.ts`:
```typescript
server: {
  port: 3001,  // Use different port
  // ...
}
```

## Next Steps

1. Customize the backend to match your EdgeSim CLI interface
2. Add authentication if needed
3. Set up output file handling and download
4. Add visualization of simulation results
5. Deploy to production server

## File Export Checklist

- [ ] All files downloaded from Figma Make
- [ ] Directory structure created
- [ ] Node.js installed
- [ ] `npm install` completed successfully
- [ ] Backend created (if using API mode)
- [ ] Anaconda environment set up
- [ ] Frontend runs at http://localhost:3000
- [ ] Backend runs at http://localhost:8000 (if applicable)
- [ ] Test simulation executed successfully

## Support

For EdgeSim-specific issues, refer to your EdgeSim documentation.
For frontend issues, check:
- Browser console for errors
- Terminal output from `npm run dev`
- Backend logs (if using API mode)
