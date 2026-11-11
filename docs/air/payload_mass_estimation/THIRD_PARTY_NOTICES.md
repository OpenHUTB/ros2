# Third-party model

`models/community_quadrotor.xml` is copied without modification from OpenHUTB/ros2 community PR #178, repository eeexty/ros2, commit ee3227230d9eb6bd571152198e3f1ed76a7276b9, path src/air/uav_mujoco_sim/models/quadrotor.xml. The source package declares MIT; its original XML comments are retained. Exact hash and source record are in models/source.json. The PR was open when the model was first reused.

The new simulation changes the central body mass at runtime to represent an added payload. Data, controller, estimators and plots in this project are newly produced. No training data or metrics are copied from the source project.
