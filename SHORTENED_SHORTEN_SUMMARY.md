# Kolleg CellML Integration Summary

## Overview
This document summarizes the steps taken to integrate the `kolleg.cellml` file into the AMI (Agonist-antagonist Myoneural Interface) simulation project, replacing the previous `shorten.cellml` model.

## Changes Made

### 1. CellML File Structure Analysis
The `kolleg.cellml` file was analyzed to determine its structure:
- **States**: ~56 state variables (differential equations)
  - Membrane potentials: `vS`, `vT` (wal_environment)
  - Ion concentrations: `K_i`, `K_t`, `K_e`, `Na_i`, `Na_t`, `Na_e` (wal_environment)
  - Channel gating variables: `n`, `h_K`, `m`, `h`, `S`, `n_t`, `h_K_t`, `m_t`, `h_t`, `S_t`
  - Calcium release: `C_0-C_4`, `O_0-O_4` (sternrios component)
  - Calcium dynamics and crossbridge kinetics: ~28 variables (razumova component)
- **Algebraics**: ~71 algebraic variables (calculated variables)

### 2. Source Code Updates

#### A. muscle.cpp
- Added new `#define Kolleg` option
- Set `N_STATES = 56` and `N_ALGEBRAICS = 71` for Kolleg model
- Changed active model from `#define Shorten` to `#define Kolleg`

#### B. variables.py
- Updated `cellml_file` path to point to `kolleg.cellml`:
  ```python
  cellml_file = input_directory+"/kolleg.cellml"
  ```

#### C. helper.py
- Added new mapping section for "kolleg" CellML model:
  ```python
  elif "kolleg" in variables.cellml_file:
    variables.muscle1_mappings = {
      ("parameter", 0): ("algebraic", "wal_environment/I_HH"),  # stimulation current
      ("parameter", 1): ("constant", "razumova/L_x"),          # fiber stretch λ
      ("connectorSlot", 0): ("state", "wal_environment/vS"),   # membrane voltage
    }
    variables.parameters_initial_values = [0.0, 1.0]          # I_HH=0, λ=1
    variables.nodal_stimulation_current = 1200.               # stimulation amplitude
    variables.vm_value_stimulated = 40.                       # stimulated voltage
  ```

### 3. Key Differences from Shorten Model
The kolleg.cellml model is based on the Shorten model but includes:
- Full electrophysiology with sarcolemma and t-tubule membranes
- Detailed ion channel descriptions (K+, Cl-, Na+ currents)
- Calcium dynamics with multiple buffering systems
- Crossbridge kinetics following Razumova formulation
- Stern-Rios calcium release model

### 4. Parameter Mappings
- **Parameter 0**: `wal_environment/I_HH` - stimulation current (algebraic variable)
- **Parameter 1**: `razumova/L_x` - fiber stretch parameter (constant)
- **Connector Slot 0**: `wal_environment/vS` - sarcolemma membrane voltage (state variable)

### 5. Build Instructions
After making these changes, rebuild the project:
```bash
cd build_release
make muscle
```
or using the OpenDiHu build system:
```bash
mkorn && sr
```

### 6. Testing
Run the simulation with:
```bash
./muscle ../settings_muscle_left.py
```

## Technical Notes
- The kolleg.cellml model maintains compatibility with the existing AMI framework
- State and algebraic counts were determined by analyzing the CellML file structure
- Variable mappings follow the same pattern as the shorten model but use kolleg-specific component names
- The model supports both electrophysiology and mechanical coupling through the fiber stretch parameter

## Expected Behavior
The kolleg model should provide:
- More detailed electrophysiological responses
- Improved calcium dynamics modeling
- Enhanced muscle fatigue representation
- Better representation of excitation-contraction coupling

## Troubleshooting
If compilation errors occur:
1. Verify CellML file is present at the specified path
2. Check that N_STATES and N_ALGEBRAICS match the compiled model
3. Ensure variable names in mappings match those in the CellML file
4. Rebuild the project completely if needed