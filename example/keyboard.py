try:
    import isaacsim
except ImportError:
    pass
import sys,os
import torch
import socket
import time
import argparse

# Paths
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../isaac_sim')))

# Argument parser
parser = argparse.ArgumentParser()
parser.add_argument(
    "--headless_mode",
    type=str,
    default=None,
    help="To run headless, use one of [native, websocket], webrtc might not work.",
)
parser.add_argument("--robot", type=str, default="m1013.yml", help="robot configuration to load")
parser.add_argument(
    "--external_asset_path",
    type=str,
    default=None,
    help="Path to external assets when loading an externally located robot",
)
parser.add_argument(
    "--external_robot_configs_path",
    type=str,
    default=None,
    help="Path to external robot config when loading an external robot",
)
parser.add_argument(
    "--visualize_spheres",
    action="store_true",
    help="When True, visualizes robot spheres",
    default=False,
)
parser.add_argument(
    "--reactive",
    action="store_true",
    help="When True, runs in reactive mode",
    default=False,
)
parser.add_argument(
    "--constrain_grasp_approach",
    action="store_true",
    help="When True, approaches grasp with fixed orientation and motion only along z axis.",
    default=False,
)
parser.add_argument(
    "--reach_partial_pose",
    nargs=6,
    metavar=("qx", "qy", "qz", "x", "y", "z"),
    help="Reach partial pose",
    type=float,
    default=None,
)
parser.add_argument(
    "--hold_partial_pose",
    nargs=6,
    metavar=("qx", "qy", "qz", "x", "y", "z"),
    help="Hold partial pose while moving to goal",
    type=float,
    default=None,
)
args = parser.parse_args()

# Simulation app
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(
    {
        "headless": args.headless_mode is not None,
        "width": "1920",
        "height": "1080",
    }
)

# Imports for functions and utilities
import carb
import numpy as np
from helper import add_extensions, add_robot_to_scene
from omni.isaac.core import World
import omni.kit.commands
from omni.isaac.core.objects import cuboid, sphere
from pxr import Sdf

# OV Imports
import omni.usd
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.materials import OmniPBR
from omni.isaac.core.prims import XFormPrim
import omni.ui as ui
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import Cuboid, Mesh
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.types.state import JointState
from curobo.util.logger import log_error, setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import (
    get_assets_path,
    get_filename,
    get_path_of_dir,
    get_robot_configs_path,
    get_world_configs_path,
    join_path,
    load_yaml,
)
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
    MotionGenStatus,
    PoseCostMetric,
)

# Function to check if the robot has stopped moving
def is_robot_static(sim_js, threshold=0.2):
    return np.max(np.abs(sim_js.velocities)) < threshold


import random
from omni.isaac.debug_draw import _debug_draw
import pygame

# Initialize Pygame
pygame.init()

# Set up screen (just for initializing Pygame)
screen = pygame.display.set_mode((100, 100))
# Movement increments for x, y, z axes
movement_increment = 0.01  # Adjust movement speed
# Define movement keys
KEYS = {
    pygame.K_w: (0, 0, movement_increment),  # Move along Z axis (forward)
    pygame.K_s: (0, 0, -movement_increment),  # Move along Z axis (backward)
    pygame.K_a: (-movement_increment, 0, 0),  # Move along X axis (left)
    pygame.K_d: (movement_increment, 0, 0),  # Move along X axis (right)
    pygame.K_q: (0, movement_increment, 0),  # Move along Y axis (forward)
    pygame.K_e: (0, -movement_increment, 0),  # Move along Y axis (backward)
    pygame.K_f: (0, 0, 0),  # Placeholder for command complete (Enter key)
}

target_material = OmniPBR("/World/looks/t", color=np.array([1, 0, 1]))

def is_valid_array(x):
    return isinstance(x, np.ndarray) and x.dtype != object and x is not None

def draw_points(rollouts: torch.Tensor):
    if rollouts is None:
        return
    draw = _debug_draw.acquire_debug_draw_interface()
    N = 100
    # if draw.get_num_points() > 0:
    draw.clear_points()
    cpu_rollouts = rollouts.cpu().numpy()
    b, h, _ = cpu_rollouts.shape
    point_list = []
    colors = []
    for i in range(b):
        # get list of points:
        point_list += [
            (cpu_rollouts[i, j, 0], cpu_rollouts[i, j, 1], cpu_rollouts[i, j, 2]) for j in range(h)
        ]
        colors += [(1.0 - (i + 1.0 / b), 0.3 * (i + 1.0 / b), 0.0, 0.3) for _ in range(h)]
    sizes = [10.0 for _ in range(b * h)]
    draw.draw_points(point_list, colors, sizes)

def main():
    print("Waiting for connection...")
    print("Connect to DRFL first and move the robot to its initial pose")

    # If you want to modify the robot's initial position,
    # please edit the value of 'retract_contig' in robot.yml+

    # When running in simulation mode without connecting to the API, do not enter the host IP.
    print("Running in simulation mode. Do not enter the host IP.")
    host = input("Enter host IP (or press ENTER to continue without connection): ").strip()
    if host:
        port = 5005
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((host, port))
        server_socket.listen(1)
        server_socket.settimeout(1.0)

        try:
            conn, addr = server_socket.accept()
            print("Connected by", addr)
        except socket.timeout:
            print("No connection within timeout, continuing...")
            conn = None # Set to None in case of connection failure
    else:
        conn = None  # No host provided, so no connection

    my_world = World(stage_units_in_meters=1.0)
    stage = my_world.stage
    target = Mesh(
        name="target",
        file_path=join_path(get_assets_path(), "/isaac-sim/kit/python/lib/python3.10/site-packages/curobo/content/configs/robot/tool.obj"),
        pose=[0.5, -0.5, 0.5, 0, 1.0, 0, 0],
        color=[0.0, 1.0, 0.0, 0.8],
        scale=[1.08,1.08,1.1]
    )
    target_2 = cuboid.VisualCuboid(
        "/World/target_2",
        position=np.array([0.6, 0.055, 0.7]),
        orientation=np.array([0.0, 1.0, 0.0, 1.0]),
        size=0.04,
        color=np.array([0, 0, 1.0]),
    )
    move_target = cuboid.VisualCuboid(
        "/World/move_target",
        position=np.array([0.5, -0.5, 0.6]),
        orientation=np.array([0, 1.0, 0, 0.0]),
        visual_material=target_material,
        color=np.array([1.0, 0, 0]),
        size=0.05,
        scale=[1.3, 1.0, 1.0],
    )

    following_target = True

    setup_curobo_logger("warn")
    past_pose = None
    n_obstacle_cuboids = 10
    n_obstacle_mesh = 10

    usd_help = UsdHelper()
    target_pose = None

    tensor_args = TensorDeviceType()
    robot_cfg_path = get_robot_configs_path()
    if args.external_robot_configs_path is not None:
        robot_cfg_path = args.external_robot_configs_path
    robot_cfg = load_yaml(join_path(robot_cfg_path, args.robot))["robot_cfg"]

    if args.external_asset_path is not None:
        robot_cfg["kinematics"]["external_asset_path"] = args.external_asset_path
    if args.external_robot_configs_path is not None:
        robot_cfg["kinematics"]["external_robot_configs_path"] = args.external_robot_configs_path
    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = robot_cfg["kinematics"]["cspace"]["retract_config"]

    robot, robot_prim_path = add_robot_to_scene(robot_cfg, my_world)

    articulation_controller = None

    obstacle_1 = Mesh(
        name="obstacle1",
        file_path=join_path(get_assets_path(), "/isaac-sim/kit/python/lib/python3.10/site-packages/curobo/content/configs/robot/sphere.obj"),
        pose=[0.7, -0.25, 0.45, 0, 0, 0, 1],
        color=[0.0, 0.0, 0.5, 1.0],
        scale=[1.5,1.5,1.5]
    )
    obstacle_2 = Mesh(
        name="obstacle2",
        file_path=join_path(get_assets_path(), "/isaac-sim/kit/python/lib/python3.10/site-packages/curobo/content/configs/robot/sphere.obj"),
        pose=[0.5, 0.4, 0.55, 0, 0, 0, 1],
        color=[1.0, 0.5, 0.0, 1.0] ,
    )
    stand_1 = Mesh(
        name="stand1",
        file_path=join_path(get_assets_path(), "/isaac-sim/kit/python/lib/python3.10/site-packages/curobo/content/configs/robot/stand_large.obj"),
        pose=[1, 0.4, -0.05, -0.5, -0.5,  0.5,  0.5],
        color=[0.1, 0.1, 0.1, 1.0] ,
    )
    stand_2 = Mesh(
        name="stand2",
        file_path=join_path(get_assets_path(), "/isaac-sim/kit/python/lib/python3.10/site-packages/curobo/content/configs/robot/stand_small.obj"),
        pose=[1, -0.25, -0.05, -0.5, -0.5,  0.5,  0.5],
        color=[0.1, 0.1, 0.1, 1.0] ,
    )
    wall_obstacle = Cuboid(
        name= "table", 
        pose=[0.0, 0.0, -0.06, 1, 0, 0, 0.0], 
        dims=[2, 1.6, 0.13], 
        color=[0.3, 0.3, 0.3, 1.0])
    wall_obstacle2 = Cuboid(
        name="wall", 
        pose=[-0.65, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0], 
        dims=[0.05, 1.6, 2.0], 
        color=[0.3, 0.3, 0.3, 1.0])

    # Merge into a WorldConfig (cuboid and mesh handled separately)
    world_cfg = WorldConfig(
        mesh = [obstacle_1, obstacle_2, stand_1, stand_2],
        cuboid =[wall_obstacle, wall_obstacle2], # Add obstacles to world (visual + logical)
    )

    trajopt_dt = None
    optimize_dt = True
    trajopt_tsteps = 32
    trim_steps = None
    max_attempts = 4
    interpolation_dt = 0.03
    enable_finetune_trajopt = True
    if args.reactive:
        trajopt_tsteps = 40
        trajopt_dt = 0.04
        optimize_dt = False
        max_attempts = 1
        trim_steps = [1, None]
        interpolation_dt = trajopt_dt
        enable_finetune_trajopt = False
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_cfg,
        world_cfg,
        tensor_args,
        collision_checker_type=CollisionCheckerType.MESH,
        num_trajopt_seeds=4,
        num_graph_seeds=0,
        interpolation_dt=interpolation_dt,
        collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
        optimize_dt=optimize_dt,
        trajopt_dt=trajopt_dt,
        trajopt_tsteps=trajopt_tsteps,
        trim_steps=trim_steps,
        position_threshold=1.0,
        rotation_threshold=1.0,
    )
    motion_gen = MotionGen(motion_gen_config)
    if not args.reactive:
        print("warming up...")
        motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)

    print("Curobo is Ready")

    add_extensions(simulation_app, args.headless_mode)

    plan_config = MotionGenPlanConfig(
        enable_graph=False,
        enable_graph_attempt=2,
        max_attempts=6,
        enable_finetune_trajopt=False,
        time_dilation_factor=0.5 if not args.reactive else 1.0,
    )

    usd_help.load_stage(my_world.stage)
    usd_help.add_world_to_stage(world_cfg, base_frame="/World")
    usd_help.add_mesh_to_stage(target, base_frame="/World")

    collision_enabled = bool(move_target.get_collision_enabled())

    if collision_enabled:
        move_target.get_collision_enabled().set(False)

    cmd_plan = None
    cmd_idx = 0
    my_world.scene.add_default_ground_plane()
    i = 0
    spheres = None
    past_cmd = None
    target_orientation = None
    already_planned = False
    last_target_position = None
    last_target_orientation = None
    cube_position, cube_orientation = target_2.get_world_pose()
    move_position, move_orientation = move_target.get_world_pose()
    interval = interpolation_dt
    next_time = time.perf_counter() + interval
    tool_prim = XFormPrim("/World/target")
    button_ = False

    while simulation_app.is_running():
        my_world.step(render=True)
        if not my_world.is_playing():
            if i % 100 == 0:
                print("**** Click Play to start simulation *****")
                if conn is None and host:
                    try:
                        conn, addr = server_socket.accept()
                        print("Connected by", addr)
                    except socket.timeout:
                        print("No connection within timeout, continuing...")
                        conn = None  # Set to None in case of connection failure
            i += 1
            continue

        step_index = my_world.current_time_step_index
        if articulation_controller is None:
            articulation_controller = robot.get_articulation_controller()
        if step_index < 2:
            my_world.reset()
            robot._articulation_view.initialize()
            idx_list = [robot.get_dof_index(x) for x in j_names]
            robot.set_joint_positions(default_config, idx_list)

            robot._articulation_view.set_max_efforts(
                values=np.array([5000 for i in range(len(idx_list))]), joint_indices=idx_list
            )
        if step_index < 20:
            continue

        if step_index == 50 or step_index % 1000 == 0.0:
            obstacles = usd_help.get_obstacles_from_stage(
                reference_prim_path=robot_prim_path,
                ignore_substring=[
                    robot_prim_path,
                    "/World/target",
                    "/World/target_2",
                    "/World/move_target",
                    "/World/defaultGroundPlane",
                    "/curobo",
                ],
            ).get_collision_check_world()

            motion_gen.update_world(obstacles)
            carb.log_info("Synced CuRobo world from stage.")

        tool_position, tool_orientation = tool_prim.get_world_pose()
        cube_position, cube_orientation = target_2.get_world_pose()
        move_position, move_orientation = move_target.get_world_pose()
        already_planned = False
        moved = False 
        dx, dy, dz = 0, 0, 0
        # Poll events from the event queue
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_f:
                    moved = False
                    button_ = True
                    move_position, move_orientation = move_target.get_world_pose()
                    tool_prim.set_world_pose(move_position, move_orientation)
                    continue  

            keys = pygame.key.get_pressed()
            for key in KEYS:
                if keys[key]:
                    delta = KEYS[key]
                    dx += delta[0]
                    dy += delta[1]
                    dz += delta[2]
                    moved = True

            if moved:
                move_position, move_orientation = move_target.get_world_pose()
                new_position = np.array(move_position) + np.array([dx, dy, dz])
                move_target.set_world_pose(new_position, move_orientation)
     

        sim_js = robot.get_joints_state()
        sim_js_names = robot.dof_names
        robot_static = is_robot_static(sim_js)

        cu_js = JointState(
            position=tensor_args.to_device(sim_js.positions),
            velocity=tensor_args.to_device(sim_js.velocities),
            acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=sim_js_names,
        )
        
        if not args.reactive:
            cu_js.velocity *= 0.0
            cu_js.acceleration *= 0.0

        if args.reactive and past_cmd is not None:
            cu_js.position[:] = past_cmd.position
            cu_js.velocity[:] = past_cmd.velocity
            cu_js.acceleration[:] = past_cmd.acceleration
        cu_js = cu_js.get_ordered_joint_state(motion_gen.kinematics.joint_names)

        if args.visualize_spheres and step_index % 2 == 0:
            sph_list = motion_gen.kinematics.get_robot_as_spheres(cu_js.position)
            if spheres is None:
                spheres = []
                for si, s in enumerate(sph_list[0]):
                    sp = sphere.VisualSphere(
                        prim_path="/curobo/robot_sphere_" + str(si),
                        position=np.ravel(s.position),
                        radius=float(s.radius),
                        color=np.array([0, 0.8, 0.2]),
                    )
                    spheres.append(sp)
            else:
                for si, s in enumerate(sph_list[0]):
                    if not np.isnantarget_moved(s.position[0]):
                        spheres[si].set_world_pose(position=np.ravel(s.position))
                        spheres[si].set_radius(float(s.radius))


        if not moved and button_: 
            if is_valid_array(past_pose) and is_valid_array(past_orientation) and \
                is_valid_array(last_target_position) and is_valid_array(last_target_orientation):
                if (np.linalg.norm(past_pose - last_target_position) <= 0.001) and (np.linalg.norm(past_orientation - last_target_orientation) <= 0.0001):
                    # Skip replan if same as previous target
                    already_planned = False
                    print(">>> Skip replan - same as past pose.")

            if last_target_position is not None and last_target_orientation is not None:
                position_tensor = torch.tensor(last_target_position, dtype=torch.float32).reshape(1, 3)
                quaternion_tensor = torch.tensor(last_target_orientation, dtype=torch.float32).reshape(1, 4)
                
                # Then construct Pose
                ik_goal = Pose(
                    position=tensor_args.to_device(position_tensor),
                    quaternion=tensor_args.to_device(quaternion_tensor),
                )
                plan_config.check_start_validity = False

                result = motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, plan_config)

                succ = result.success.item()

                if succ:
                    cmd_plan = result.get_interpolated_plan()
                    cmd_plan = motion_gen.get_full_js(cmd_plan)

                    ee_positions = []
                    # Extract tensor of joint positions
                    positions = cmd_plan.position  # torch.Size([62, 6])
                    # Iterate over each row (joint position at each timestep)
                    for i in range(positions.shape[0]):
                        joint_state = JointState.from_position(
                            tensor_args.to_device(positions[i].cpu().numpy()),
                            joint_names=motion_gen.rollout_fn.joint_names,
                        )
                        kin_state = motion_gen.rollout_fn.compute_kinematics(joint_state)
                        ee_pos = kin_state.ee_pos_seq[0].cpu().numpy()
                        ee_positions.append(ee_pos)
                    if len(ee_positions) > 0:
                        ee_positions_tensor = torch.tensor(ee_positions, device="cuda:0").unsqueeze(0)
                    if ee_positions_tensor is not None:
                        draw_points(ee_positions_tensor) 

                    idx_list = []
                    common_js_names = []
                    for x in sim_js_names:
                        if x in cmd_plan.joint_names:
                            idx_list.append(robot.get_dof_index(x))
                            common_js_names.append(x)

                    cmd_plan = cmd_plan.get_ordered_joint_state(common_js_names)
                    cmd_idx = 0
                    interval = interpolation_dt
        elif not already_planned and not button_:
            if (not np.array_equal(past_pose, target_pose)) and (not np.array_equal(past_orientation, target_orientation)) and robot_static:
                already_planned = False

                if following_target:
                    target_pose = cube_position
                    target_orientation = cube_orientation
                else:
                    target_pose = tool_position
                    target_orientation = tool_orientation

                following_target = not following_target

                ee_translation_goal = target_pose
                ee_orientation_teleop_goal = target_orientation

                ik_goal = Pose(
                    position=tensor_args.to_device(ee_translation_goal),
                    quaternion=tensor_args.to_device(ee_orientation_teleop_goal),
                )

                result = motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, plan_config)
                succ = result.success.item()
                if succ:
                    cmd_plan = result.get_interpolated_plan()
                    cmd_plan = motion_gen.get_full_js(cmd_plan)
                    
                    ee_positions = []
                    # Extract tensor of joint positions
                    positions = cmd_plan.position  # torch.Size([62, 6])
                    # Iterate over each row (joint position at each timestep)
                    for i in range(positions.shape[0]):
                        joint_state = JointState.from_position(
                            tensor_args.to_device(positions[i].cpu().numpy()),
                            joint_names=motion_gen.rollout_fn.joint_names,
                        )
                        kin_state = motion_gen.rollout_fn.compute_kinematics(joint_state)
                        ee_pos = kin_state.ee_pos_seq[0].cpu().numpy()
                        ee_positions.append(ee_pos)
                    if len(ee_positions) > 0:
                        ee_positions_tensor = torch.tensor(ee_positions, device="cuda:0").unsqueeze(0)
                    if ee_positions_tensor is not None:
                        draw_points(ee_positions_tensor) 

                    idx_list = []
                    common_js_names = []
                    for x in sim_js_names:
                        if x in cmd_plan.joint_names:
                            idx_list.append(robot.get_dof_index(x))
                            common_js_names.append(x)

                    cmd_plan = cmd_plan.get_ordered_joint_state(common_js_names)
                    cmd_idx = 0

            past_pose = cube_position if not following_target else tool_position
            past_orientation = cube_orientation if not following_target else tool_orientation
            already_planned = False
        
        if cmd_plan is not None:
            sim_js = robot.get_joints_state()
            sim_js_names = robot.dof_names
            robot_static = is_robot_static(sim_js)

            cu_js = JointState(
                position=tensor_args.to_device(sim_js.positions),
                velocity=tensor_args.to_device(sim_js.velocities),
                acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
                joint_names=sim_js_names,
            )
            cu_js = cu_js.get_ordered_joint_state(motion_gen.kinematics.joint_names)
            motion_gen.rollout_fn.update_start_state(JointState(position=tensor_args.to_device(sim_js.positions)))
            cmd_state = cmd_plan[cmd_idx]
            past_cmd = cmd_state.clone()
            # get joint limits
            limits = motion_gen.rollout_fn.kinematics.get_joint_limits()
            lower = limits.position[0]  # shape: [DOF]
            upper = limits.position[1]

            # clamp position to within limits
            buffer = 0.02  # radian 
            clipped_pos = torch.clamp(cmd_state.position, min=lower + buffer, max=upper - buffer)
            pos_np = clipped_pos.cpu().numpy()
            vel_np = cmd_state.velocity.cpu().numpy()
            acc_np = cmd_state.acceleration.cpu().numpy()
            merged_np = np.concatenate([pos_np, vel_np, acc_np])
            data_cmd = merged_np.tobytes()
            if conn is not None:
                conn.sendall(data_cmd)

            now = time.perf_counter()
            sleep_duration = next_time - now

            if sleep_duration > 0:
                time.sleep(sleep_duration)
            else:
                # print(f"[Delay] sendall overrun by {-sleep_duration:.6f}s")
                next_time = time.perf_counter() + interval
            next_time += interval
            art_action = ArticulationAction(
                clipped_pos.cpu().numpy(),
                cmd_state.velocity.cpu().numpy(),
                joint_indices=idx_list,
            )
            # print("[ACTION] Applying action...")
            articulation_controller.apply_action(art_action)
            cmd_idx += 1

            for _ in range(2):
                my_world.step(render=False)

            if cmd_idx >= len(cmd_plan.position):
                    cmd_idx = 0
                    cmd_plan = None
                    past_cmd = None
                    past_pose = None
                    past_orientation = None
                    already_planned = False
                    button_ = False
                    
        if cmd_plan is None and past_cmd is not None:
            print("[INFO] Holding last known pose.")
            art_action = ArticulationAction(
                past_cmd.position.cpu().numpy(),
                past_cmd.velocity.cpu().numpy(),
                joint_indices=idx_list,
            )
            articulation_controller.apply_action(art_action)
            my_world.step(render=False)   
    simulation_app.close()

if __name__ == "__main__":
    main()
