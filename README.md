[![Build Status](https://travis-ci.com/yukkysaito/amcl_3d.svg?branch=master)](https://travis-ci.com/yukkysaito/amcl_3d)

I develop this package every weekend.
The alpha version will release around mid-November

# Work in progress
90%

# Install Requirements

```
$ git clone https://github.com/yukkysaito/amcl_3d.git <catkin_ws/src path>
$ rosdep install --from-paths <amcl_3d path> -y
```

# TODO
- add prediction model (particle filter)

# Done
- design interface (particle filter)
- make ros package template
- add basic resample function (particle filter)
- add init function(particle filter)
- build check (travis ci)
- resampling timing based on ESS (effective sample size)
- add simple likelihood model for lidar measurement (particle filter)
- implement argumented mcl
- implement kld sampling

# Reference
## English Reference
### General
- http://www.probabilistic-robotics.org/
### Effective Sampling Size
- http://www.cns.nyu.edu/~eorhan/notes/particle-filtering.pdf
### Avoidance Numerical Underflow
- http://www.maths.lu.se/fileadmin/maths/forskning_research/InferPartObsProcess/particlemethods.pdf

## Japanese Reference
- http://lab.cntl.kyutech.ac.jp/~nishida/lecture/psc/no11.pdf
- https://gihyo.jp/book/2018/978-4-7741-9646-6
- https://qiita.com/MoriKen/items/dfb6eb168649873589f0
