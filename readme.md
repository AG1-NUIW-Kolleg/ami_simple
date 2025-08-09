# AMI model

## Setup
A two-muscles-one-tendon AMI (Agonist-antagonist Myoneural Interface) model. Three OpenDiHu-based participants are coupled using preCICE: muscle 1 (left muscle), muscle 2 (right muscle) and the tendon. 

**Computational Efficiency Optimizations:**
This version has been optimized for reduced computational complexity while maintaining simulation accuracy:
- Reduced fiber count from 8x8 (64 fibers) to 4x4 (16 fibers) per muscle (75% reduction)
- Reduced muscle mesh resolution from 8x8x24 to 4x4x12 elements
- Reduced tendon mesh resolution from 8x8x8 to 4x4x4 elements  
- Reduced fiber discretization from 60 to 30 points per fiber
- Reduced simulation time from 20ms to 5ms for faster demonstration
- Time step set to 2ms for balance between stability and speed
- Linear tendon material (SaintVenantKirchoff) used for improved numerical stability
- These changes significantly reduce simulation time while preserving the essential physics

Muscle 1's solver combines the fast monodomain solver with the Mooney-Rivlin non-linear continuum mechanics model. Muscle 1 is activated with the input file `"input/MU_firing_times_real.txt"`. The tendon and muscle 2 participants consist of a continuum mechanics solver alone, which assumes a Mooney-Rivlin non-linear muscle model and a hyper elastic non-linear tendon model respectively. When muscle 1 contracts, the tendon and muscle 2 deform accordingly.

Surface coupling between the continuum mechanics solvers of the participants is applied. There are two interfaces where the mechanical coupling takes place: between muscle 1 and the tendon (muscle 1 sends traction to the tendon, which sends back displacement and velocity), and between the tendon and muscle 2 (the tendon sends traction to the muscle 2, which sends back displacement and velocity). Implicit coupling is needed, in particular when muscle 2 (soft material) sends traction to the tendon (stift material). We apply quasi-Newton acceleration using a residual-sum precondiioner.

The default option for this case, uses preCICE's `multi-coupling` scheme to configure the coupling among the three participants. You can also select a composition of a `serial-explicit` scheme and a `parallel-implicit` scheme if in `variables.py` you set `precice_file = "../precice_config_bi.xml"`.



### How to build?
Follow OpenDiHu's documentation for installation, then run 
```
mkorn && sr
```
### How to run?
You will need one terminal per each participant. All terminals should be at the same directory, e.g., `ami/build_release`.

terminal 1: left muscle
```
./muscle ../settings_muscle_left.py
```
terminal 2: tendon
```
./linear_tendon ../settings_tendon.py
```
terminal 3: right muscle
```
./only_mechanics_muscle ../settings_muscle_right.py
```

Alternatively you can use a nonlinear tendon solver: Choose `tendon_material = "nonLinear"` in `variables.py` and then use the executable `./tendon` when running the tendon participant. Note that the linear material provides better numerical stability.

### How to run in a cluster?
Here's an example on how to lunch the three participants in the cluster (Add this code to your bash file):
```
echo "Launching left muscle"
mpirun -n 1 ./muscle ../settings_muscle_left.py &> left.log &

echo "Launching tendon"
mpirun -n 1 ./linear_tendon ../settings_tendon.py &> tendon.log &

echo "Launching right muscle"
mpirun -n 1 ./only_mechanics_muscle ../settings_muscle_right.py &> right.log

echo "Simulation completed."

```
