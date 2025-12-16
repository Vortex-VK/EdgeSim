
  # EdgeSim

  This is a code bundle for EdgeSim. The original project is available at https://www.figma.com/design/8VC7gf2Vd0BZdL4KoOKNSh/EdgeSim.

  ## Running the code

Run `npm i` inside `frontend/` to install the dependencies.

Run `npm run dev` to start the development server (defaults to http://localhost:3000).

## Project layout
- `frontend/src/` – React app (Text/Visual modes, UI components, styles).
- `frontend/vite.config.ts` – Vite + Tailwind CSS v4 config; proxies `/api` → `http://localhost:8000` by default.
- `frontend/tsconfig*.json` – TypeScript settings with `@` alias → `src/`.

## Next step: hook up the backend
- Start/point a FastAPI (or similar) backend at `http://localhost:8000` with `/api/*` routes that translate UI state into EdgeSim CLI calls.
- Update `frontend/vite.config.ts` proxy target if your backend runs elsewhere.
- When wiring, send the UI’s prompt/seed/profile/logging flags to the existing EdgeSim CLI (`src/edgesim/cli.py`) and return run metadata (paths to `runs/<...>` artifacts) so the UI can link to results.
  
