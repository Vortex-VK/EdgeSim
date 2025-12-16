# Backend Connection Guide

This guide shows you **exactly where** to connect your EdgeSim backend to the frontend.

## 🎯 Connection Points Overview

Currently, the frontend **only logs commands to the console**. There are **4 main connection points** where you need to add backend API calls:

1. **Text Mode - Test Run** (Line ~248-256 in App.tsx)
2. **Text Mode - Batch Run** (Line ~258-267 in App.tsx)
3. **Visual Mode - Test Run** (Line ~719-730 in App.tsx)
4. **Visual Mode - Batch Run** (Line ~732-744 in App.tsx)

---

## 📍 Connection Point #1: Text Mode - Test Run

**Location:** `/App.tsx`, lines 248-256

**Current Code:**
```typescript
const handleTestRun = () => {
  let command = `edgesim run-one "${prompt}" --seed ${seed} --gui --realtime --dt ${timestep} --slowmo ${slowmo}`;
  if (siteProfile !== "default") command += ` --site ${siteProfile}`;
  if (debugVisuals) command += ` --debug-visuals`;
  if (lidarLogging === "full") command += ` --lidar-logging`;
  if (lidarLogging === "events") command += ` --lidar-events-only`;
  console.log("Executing test run:", command);  // ← CURRENT: Just logs to console
  setShowTestDialog(false);
};
```

**Replace With (Backend Integration):**
```typescript
const handleTestRun = async () => {
  let command = `edgesim run-one "${prompt}" --seed ${seed} --gui --realtime --dt ${timestep} --slowmo ${slowmo}`;
  if (siteProfile !== "default") command += ` --site ${siteProfile}`;
  if (debugVisuals) command += ` --debug-visuals`;
  if (lidarLogging === "full") command += ` --lidar-logging`;
  if (lidarLogging === "events") command += ` --lidar-events-only`;
  
  console.log("Executing test run:", command);
  
  // ✅ ADD THIS: Call your backend API
  try {
    const response = await fetch('/api/simulate', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        mode: 'test',
        command: command,
        config: {
          scenario_type: 'text',
          text_prompt: prompt,
          seed: seed,
          timestep: timestep,
          slowmo: slowmo,
          site_profile: siteProfile,
          debug_visuals: debugVisuals,
          lidar_logging: lidarLogging,
          gui: true,
          realtime: true
        }
      })
    });
    
    const result = await response.json();
    
    if (result.success) {
      console.log('Simulation started:', result.job_id);
      // Optionally show success toast or redirect to results page
    } else {
      console.error('Simulation failed:', result.error);
      alert('Failed to start simulation: ' + result.error);
    }
  } catch (error) {
    console.error('Error calling backend:', error);
    alert('Failed to connect to backend: ' + error);
  }
  
  setShowTestDialog(false);
};
```

---

## 📍 Connection Point #2: Text Mode - Batch Run

**Location:** `/App.tsx`, lines 258-267

**Current Code:**
```typescript
const handleBatchRun = () => {
  let command = `edgesim run-batch "${prompt}" --runs ${numRuns} --seed ${seed} --profile ${batchProfile}`;
  if (siteProfile !== "default") command += ` --site ${siteProfile}`;
  if (timeBudget) command += ` --time-budget-min ${timeBudget}`;
  if (autoDegrade) command += ` --auto-degrade`;
  if (lidarLogging === "full") command += ` --lidar-logging`;
  if (lidarLogging === "events") command += ` --lidar-events-only`;
  console.log("Executing batch run:", command);  // ← CURRENT: Just logs to console
  setShowBatchDialog(false);
};
```

**Replace With (Backend Integration):**
```typescript
const handleBatchRun = async () => {
  let command = `edgesim run-batch "${prompt}" --runs ${numRuns} --seed ${seed} --profile ${batchProfile}`;
  if (siteProfile !== "default") command += ` --site ${siteProfile}`;
  if (timeBudget) command += ` --time-budget-min ${timeBudget}`;
  if (autoDegrade) command += ` --auto-degrade`;
  if (lidarLogging === "full") command += ` --lidar-logging`;
  if (lidarLogging === "events") command += ` --lidar-events-only`;
  
  console.log("Executing batch run:", command);
  
  // ✅ ADD THIS: Call your backend API
  try {
    const response = await fetch('/api/simulate', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        mode: 'batch',
        command: command,
        config: {
          scenario_type: 'text',
          text_prompt: prompt,
          num_runs: numRuns,
          seed: seed,
          batch_profile: batchProfile,
          site_profile: siteProfile,
          time_budget: timeBudget,
          auto_degrade: autoDegrade,
          lidar_logging: lidarLogging,
          gui: false,
          headless: true
        }
      })
    });
    
    const result = await response.json();
    
    if (result.success) {
      console.log('Batch simulation started:', result.job_id);
      // Optionally show success message or redirect to monitoring page
    } else {
      console.error('Batch simulation failed:', result.error);
      alert('Failed to start batch simulation: ' + result.error);
    }
  } catch (error) {
    console.error('Error calling backend:', error);
    alert('Failed to connect to backend: ' + error);
  }
  
  setShowBatchDialog(false);
};
```

---

## 📍 Connection Point #3: Visual Mode - Test Run

**Location:** `/App.tsx`, lines 719-730

**Current Code:**
```typescript
const handleTestRun = () => {
  const prompt = generatePrompt();
  let command = `edgesim run-one "${prompt}" --seed ${seed} --gui --realtime --dt ${timestep} --slowmo ${slowmo}`;
  if (siteProfile !== "default") command += ` --site ${siteProfile}`;
  if (debugVisuals) command += ` --debug-visuals`;
  if (lidarLogging === "full") command += ` --lidar-logging`;
  if (lidarLogging === "events") command += ` --lidar-events-only`;
  if (lidarBlackout) command += ` --lidar-blackout`;
  if (ghostObstacle) command += ` --ghost-obstacle`;
  console.log("Executing test run:", command);  // ← CURRENT: Just logs to console
  setShowTestDialog(false);
};
```

**Replace With (Backend Integration):**
```typescript
const handleTestRun = async () => {
  const prompt = generatePrompt();
  let command = `edgesim run-one "${prompt}" --seed ${seed} --gui --realtime --dt ${timestep} --slowmo ${slowmo}`;
  if (siteProfile !== "default") command += ` --site ${siteProfile}`;
  if (debugVisuals) command += ` --debug-visuals`;
  if (lidarLogging === "full") command += ` --lidar-logging`;
  if (lidarLogging === "events") command += ` --lidar-events-only`;
  if (lidarBlackout) command += ` --lidar-blackout`;
  if (ghostObstacle) command += ` --ghost-obstacle`;
  
  console.log("Executing test run:", command);
  
  // ✅ ADD THIS: Call your backend API
  try {
    const response = await fetch('/api/simulate', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        mode: 'test',
        command: command,
        config: {
          scenario_type: 'visual',
          visual_layout: objects,  // Array of objects from canvas
          text_prompt: prompt,     // Auto-generated from visual layout
          seed: seed,
          timestep: timestep,
          slowmo: slowmo,
          site_profile: siteProfile,
          debug_visuals: debugVisuals,
          lidar_logging: lidarLogging,
          lidar_blackout: lidarBlackout,
          ghost_obstacle: ghostObstacle,
          gui: true,
          realtime: true
        }
      })
    });
    
    const result = await response.json();
    
    if (result.success) {
      console.log('Simulation started:', result.job_id);
    } else {
      console.error('Simulation failed:', result.error);
      alert('Failed to start simulation: ' + result.error);
    }
  } catch (error) {
    console.error('Error calling backend:', error);
    alert('Failed to connect to backend: ' + error);
  }
  
  setShowTestDialog(false);
};
```

---

## 📍 Connection Point #4: Visual Mode - Batch Run

**Location:** `/App.tsx`, lines 732-744

**Current Code:**
```typescript
const handleBatchRun = () => {
  const prompt = generatePrompt();
  let command = `edgesim run-batch "${prompt}" --runs ${numRuns} --seed ${seed} --profile ${batchProfile}`;
  if (siteProfile !== "default") command += ` --site ${siteProfile}`;
  if (timeBudget) command += ` --time-budget-min ${timeBudget}`;
  if (autoDegrade) command += ` --auto-degrade`;
  if (lidarLogging === "full") command += ` --lidar-logging`;
  if (lidarLogging === "events") command += ` --lidar-events-only`;
  if (lidarBlackout) command += ` --lidar-blackout`;
  if (ghostObstacle) command += ` --ghost-obstacle`;
  console.log("Executing batch run:", command);  // ← CURRENT: Just logs to console
  setShowBatchDialog(false);
};
```

**Replace With (Backend Integration):**
```typescript
const handleBatchRun = async () => {
  const prompt = generatePrompt();
  let command = `edgesim run-batch "${prompt}" --runs ${numRuns} --seed ${seed} --profile ${batchProfile}`;
  if (siteProfile !== "default") command += ` --site ${siteProfile}`;
  if (timeBudget) command += ` --time-budget-min ${timeBudget}`;
  if (autoDegrade) command += ` --auto-degrade`;
  if (lidarLogging === "full") command += ` --lidar-logging`;
  if (lidarLogging === "events") command += ` --lidar-events-only`;
  if (lidarBlackout) command += ` --lidar-blackout`;
  if (ghostObstacle) command += ` --ghost-obstacle`;
  
  console.log("Executing batch run:", command);
  
  // ✅ ADD THIS: Call your backend API
  try {
    const response = await fetch('/api/simulate', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        mode: 'batch',
        command: command,
        config: {
          scenario_type: 'visual',
          visual_layout: objects,  // Array of objects from canvas
          text_prompt: prompt,     // Auto-generated from visual layout
          num_runs: numRuns,
          seed: seed,
          batch_profile: batchProfile,
          site_profile: siteProfile,
          time_budget: timeBudget,
          auto_degrade: autoDegrade,
          lidar_logging: lidarLogging,
          lidar_blackout: lidarBlackout,
          ghost_obstacle: ghostObstacle,
          gui: false,
          headless: true
        }
      })
    });
    
    const result = await response.json();
    
    if (result.success) {
      console.log('Batch simulation started:', result.job_id);
    } else {
      console.error('Batch simulation failed:', result.error);
      alert('Failed to start batch simulation: ' + result.error);
    }
  } catch (error) {
    console.error('Error calling backend:', error);
    alert('Failed to connect to backend: ' + error);
  }
  
  setShowBatchDialog(false);
};
```

---

## 🔧 How to Apply These Changes

### Option 1: Manual Find & Replace
1. Open `/App.tsx` in your code editor
2. Search for each `handleTestRun` and `handleBatchRun` function
3. Replace them with the backend-integrated versions above

### Option 2: Use the Line Numbers
1. Go to line 248 in App.tsx (Text Mode - handleTestRun)
2. Go to line 258 in App.tsx (Text Mode - handleBatchRun)
3. Go to line 719 in App.tsx (Visual Mode - handleTestRun)
4. Go to line 732 in App.tsx (Visual Mode - handleBatchRun)

---

## 🔌 Backend API Specification

Your backend should implement this endpoint:

### POST `/api/simulate`

**Request Body:**
```typescript
{
  mode: 'test' | 'batch',
  command: string,  // The complete CLI command (for logging/debugging)
  config: {
    scenario_type: 'text' | 'visual',
    text_prompt?: string,           // Text description (always sent)
    visual_layout?: Array<{         // Visual objects (only in visual mode)
      id: string,
      type: string,
      coords: Array<{x: number, y: number}>,
      label?: string
    }>,
    
    // Common parameters
    seed: number,
    timestep: number,
    slowmo: number,
    site_profile: string,
    debug_visuals: boolean,
    lidar_logging: 'none' | 'events' | 'full',
    gui: boolean,
    realtime: boolean,
    
    // Batch-only parameters
    num_runs?: number,
    batch_profile?: string,
    time_budget?: number,
    auto_degrade?: boolean,
    
    // Visual mode sensor injectors
    lidar_blackout?: boolean,
    ghost_obstacle?: boolean
  }
}
```

**Response:**
```typescript
{
  success: boolean,
  job_id?: string,        // If successful
  message?: string,       // Success message
  error?: string          // If failed
}
```

---

## 🐍 Example Python Backend (FastAPI)

Create `backend/main.py`:

```python
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import subprocess
import uuid
import os

app = FastAPI()

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class Coordinate(BaseModel):
    x: float
    y: float

class VisualObject(BaseModel):
    id: str
    type: str
    coords: List[Coordinate]
    label: Optional[str] = None

class SimulationConfig(BaseModel):
    scenario_type: str
    text_prompt: Optional[str] = None
    visual_layout: Optional[List[VisualObject]] = None
    seed: int
    timestep: Optional[float] = None
    slowmo: Optional[float] = None
    site_profile: Optional[str] = None
    debug_visuals: Optional[bool] = False
    lidar_logging: Optional[str] = 'none'
    gui: Optional[bool] = False
    realtime: Optional[bool] = False
    num_runs: Optional[int] = None
    batch_profile: Optional[str] = None
    time_budget: Optional[int] = None
    auto_degrade: Optional[bool] = False
    lidar_blackout: Optional[bool] = False
    ghost_obstacle: Optional[bool] = False

class SimulationRequest(BaseModel):
    mode: str  # 'test' or 'batch'
    command: str
    config: SimulationConfig

@app.post("/api/simulate")
async def run_simulation(request: SimulationRequest):
    """Execute EdgeSim simulation"""
    job_id = str(uuid.uuid4())
    
    try:
        # Build EdgeSim command
        if request.mode == "test":
            cmd = ["python", "-m", "edgesim", "run-one"]
        else:
            cmd = ["python", "-m", "edgesim", "run-batch"]
        
        # Add the scenario (text prompt)
        cmd.append(request.config.text_prompt)
        
        # Add common parameters
        cmd.extend(["--seed", str(request.config.seed)])
        
        if request.mode == "test":
            cmd.extend(["--gui", "--realtime"])
            if request.config.timestep:
                cmd.extend(["--dt", str(request.config.timestep)])
            if request.config.slowmo:
                cmd.extend(["--slowmo", str(request.config.slowmo)])
        else:
            cmd.extend(["--runs", str(request.config.num_runs)])
            cmd.extend(["--profile", request.config.batch_profile])
        
        # Add optional parameters
        if request.config.site_profile and request.config.site_profile != "default":
            cmd.extend(["--site", request.config.site_profile])
        
        if request.config.debug_visuals:
            cmd.append("--debug-visuals")
        
        if request.config.lidar_logging == "full":
            cmd.append("--lidar-logging")
        elif request.config.lidar_logging == "events":
            cmd.append("--lidar-events-only")
        
        if request.config.time_budget:
            cmd.extend(["--time-budget-min", str(request.config.time_budget)])
        
        if request.config.auto_degrade:
            cmd.append("--auto-degrade")
        
        if request.config.lidar_blackout:
            cmd.append("--lidar-blackout")
        
        if request.config.ghost_obstacle:
            cmd.append("--ghost-obstacle")
        
        # Execute the command
        print(f"Executing: {' '.join(cmd)}")
        
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        return {
            "success": True,
            "job_id": job_id,
            "message": f"{request.mode.capitalize()} simulation started successfully",
            "command": ' '.join(cmd)
        }
        
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to start simulation: {str(e)}"
        )

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

Run it:
```bash
conda activate your_edgesim_env
cd backend
python main.py
```

---

## ✅ Testing Your Integration

1. **Start your backend:**
   ```bash
   conda activate edgesim
   python backend/main.py
   ```

2. **Start your frontend:**
   ```bash
   npm run dev
   ```

3. **Test the flow:**
   - Open http://localhost:3000
   - Choose Text Mode or Visual Mode
   - Configure a scenario
   - Click "Test Run" or "Create Data"
   - Check browser console for API responses
   - Check backend terminal for EdgeSim execution

---

## 🎯 Quick Summary

**4 Functions to Replace:**
1. Line ~248: Text Mode `handleTestRun`
2. Line ~258: Text Mode `handleBatchRun`
3. Line ~719: Visual Mode `handleTestRun`
4. Line ~732: Visual Mode `handleBatchRun`

**What to Add:**
- Change functions to `async`
- Add `fetch('/api/simulate', {...})` call
- Send `mode`, `command`, and `config` in request body
- Handle response with `result.success` and `result.job_id`
- Add error handling with try/catch

**Backend Requirements:**
- Single endpoint: `POST /api/simulate`
- Accept the config object
- Execute EdgeSim CLI
- Return `{success, job_id, message}` or `{success: false, error}`

That's it! You now have a complete guide to connecting your EdgeSim backend. 🚀
