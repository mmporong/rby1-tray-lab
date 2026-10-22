# Third-party notices

## MuJoCo Menagerie RB-Y1

- Source: https://github.com/google-deepmind/mujoco_menagerie
- Revision: `cda5236f98383f6fbaa4820c6b18dba892cd087f`
- Model: `rainbow_robotics_rby1/rby1a_1.2.xml`
- License: Apache License, Version 2.0; full text in [third_party/mujoco_menagerie/LICENSE](third_party/mujoco_menagerie/LICENSE)

Robot model descriptions in saved `scene.xml` files derive from the above model. The scene generator changes the base fixture, structural collision exclusions, finger contact proxies, actuator coupling solver parameters, and adds the task environment. Source assets are fetched separately without redistribution in this repository. Rendered images and animations use that model. Other source credits in the upstream model package remain governed by its license.

Project code is licensed under the root MIT license. This license does not replace the upstream model license.
