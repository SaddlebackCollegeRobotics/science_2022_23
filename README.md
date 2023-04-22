# science_2022_23

#### General build:
```bash
make          # colcon build all
make clean    # clean all
```

#### Run:
```bash
make srv      # run server that will take actions upon request
```
```bash
make cli c={} t={} # run client that will send request to server
```
where `c` is the command and `t` is the target/time. `t` should be defaulted to 0 if usage is not specified (for `c` = 1,2).

#### Commands (`c`):
- `1` - Lower Lowering Platform for UV Experiment
- `2` - Raise Lowering Platform for UV Experiment
- `3` - Start Water Pump/Dispenser *(for `t` seconds)*
- `4` - Start Vacuum Pump *(for `t` seconds)*
- `5` - Rotate Funnel Cake into position with given targeted index `t`:*[0,1,2,3,4]*


#### Example run:
Terminal 1 (on Raspberry Pi):
```bash
make srv
```
Terminal 2 (on your local garbage machine):
```bash
make cli c=1 t=0    # Lower Lowering Platform for UV Experiment

make cli c=2 t=0    # Raise Lowering Platform for UV Experiment

make cli c=3 t=10   # Start Water Pump/Dispenser for 10 seconds

make cli c=4 t=69   # Start Vacuum Pump for 69 seconds

make cli c=5 t=2    # Rotate Funnel Cake into position with index 2 (3rd position) We are using 0-indexing

make cli c=5 t=0    # Rotate Funnel Cake into position with index 0 (1st position)
```

### Recording
```bash
make record
```