# Object Permanence Filter for Robust Tracking with Interactive Robots

Code for paper: Arxiv

## Abstract

Object permanence, which refers to the concept that objects continue to exist even when they are no longer perceivable through the senses, is a crucial aspect of human cognitive development. In this work, we seek to incorporate this understanding into interactive robots by proposing a set of assumptions and rules to represent object permanence in multi-object, multi-agent interactive scenarios. We integrate these rules into the particle filter, resulting in the Object Perma- nence Filter (OPF). For multi-object scenarios, we propose an ensemble of K interconnected OPFs, where each filter predicts plausible object tracks that are resilient to missing, noisy, and kinematically or dynamically infeasible measurements. Through several interactive scenarios, we demonstrate that the proposed OPF approach provides robust tracking in human-robot interactive tasks agnostic to measurement type, even in the presence of prolonged and complete occlusion.

## Usage

Initialize the OPF:

```
obj_OPF = OPF_3d(num_particles=5000, name=NAME)
```

Measurement should be of the form: `measurement = np.array([x, y, z, \theta, \phi, \psi])`, with the first three as translation and the latter three as Euler angles.

Then follow the sequence as the standard particle filter:
```
obj.predict()
obj.update(measurement)
obj.systematic_resample()
obj.resample_from_index()
```

Examples will be updated shortly.
