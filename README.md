MJCF and URDF model viewer
============

This is simple [MuJoCo XML file(MJCF)](http://mujoco.org/book/modeling.html#CURDF) and [URDF XML file](http://wiki.ros.org/en/urdf/Tutorials)viewer using [Roboschool](https://github.com/openai/roboschool).

How to use
----------

### MuJoCo XML file

Please edit bottom of `mjcf_viewer.py`.

* `model_xml` is path of MJCF model file
* `robot_name` is root body
* `foot_list` is body of contacting to the ground.

```
    foot_list = ['front_left_foot', 'front_right_foot', 'left_back_foot', 'right_back_foot']
    run(model_xml="ant.xml", robot_name="torso", foot_list=foot_list)
```

### URDF XML file

Please edit bottom of `urdf_viewer.py`.

* `model_urdf` is path of URDF model file
* `robot_name` is root body
* `foot_list` is body of contacting to the ground.

```
    foot_list = []
    run(model_urdf="atlas_description/urdf/atlas_v4_with_multisense.urdf",
        robot_name="pelvis", footlist=foot_list)
```

LICENSE
--------

The MIT License

Copyright (c) 2018 ikeyasu (http://ikeyasu.com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
