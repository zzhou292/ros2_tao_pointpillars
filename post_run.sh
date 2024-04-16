/model/pointpillars_model/tao-converter  -k tlt_encode \
               -e /model/pointpillars_model/trt.engine \
               -p points,1x204800x4,1x204800x4,1x204800x4 \
               -p num_points,1,1,1 \
               -t fp16 \
               /model/pointpillars_model/pointpillars_deployable.etlt

colcon build \
        --merge-install \
        --packages-select pp_infer \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda
        
colcon build --packages-select brake_control --merge-install

# run everything sudo

ros2 launch pp_infer pp_infer_launch.py
CMD ros2 run brake_control vehicle_detector
