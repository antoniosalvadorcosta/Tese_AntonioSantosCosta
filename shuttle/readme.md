# Instruction on how to use this shuttle drone simulator

- Choose the correspondent simulation scenario script `drone_init_(n).m` ('n' is the number of the scenario you want to run),
in script `drone_main_simul.m`
- Run main simulation script `drone_main_simul.m`:
    . Scenario 1: Mellinger controller, all aerodynamic conditions, step signal
    . Scenario 2: Mellinger controller, all aerodynamic conditions, circle trajectory, v_ref = 0, a_ref = 0
    . Scenario 3: Mellinger controller, all aerodynamic conditions, circle trajectory, derivative inputs
    . Scenario 4: Mellinger controller + rotor drag compensaton, circle trajectory, derivative inputs
    . Scenario 5: Mellinger controller + rotor drag and frame drag compensaton, circle trajectory, derivative inputs
- Plots of the results are made using the script `drone_show_data.m`

## Acknowledgments and copyright

Project Capture team
Bruno Guerreiro (bj.guerreiro@fct.unl.pt)


