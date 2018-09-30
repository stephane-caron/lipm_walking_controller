{
  //
  // State machine
  //

  "init": "Initial",
  "states": {},
  "configs": {},
  "transitions":
  [
    ["Initial", "Standing", "Standing"],
    ["Standing", "DoubleSupport", "DoubleSupport"],
    ["DoubleSupport", "SingleSupport", "SingleSupport"],
    ["DoubleSupport", "Standing", "Standing"],
    ["SingleSupport", "DoubleSupport", "DoubleSupport"]
  ],

  //
  // Tasks
  //

  "constraints":
  [
    {
      "type": "contact"
    },
    {
      "type": "kinematics", // constraint type
      "damper": [           // see Equation (3.6) in J. Vaillant's thesis
          0.1,              // interaction distance (d_i)
          0.01,             // safety distance (d_s)
          0.5],             // damper offset (xi_off)
      "robotIndex": 0,      // applies to main robot
    }
  ],
  "collisions":
  [
    {
      "type": "collision",
      "r1Index": 0,
      "r2Index": 0,
      "useMinimal": true
    }
  ],
  "contacts": [],
  "robots":
  {
    "ground":
    {
      "module": "env",
      "params": ["@AROBASE@MC_ENV_DESCRIPTION@AROBASE@", "ground"]
    }
  },

  //
  // mc_control::fsm::Controller configuration
  // 

  // Disable floating-base update from MCGlobalController
  "UpdateRealFromSensors": false,

  // When true, the FSM transitions are managed by an external tool
  "Managed": false,

  // When true and the FSM is self-managed, transitions should be triggered
  "StepByStep": false,

  // Where to look for state libraries
  "StatesLibraries": ["@MC_RTC_LIBDIR@/mc_controller/lipm_walking_controller/states"],

  // Where to look for state files
  "StatesFiles": [],

  // When true, state factory will be more verbose
  "VerboseStateFactory": true,

  // Controller is created in a sandbox, which in between a thread and a fork;
  // try to keep it to false, as it can create weird conflicts with threads
  "UseSandbox" : false,

  //
  // HRP-4
  //

  "sole":
  {
    "half_length": 0.112,
    "half_width": 0.065,
    "friction": 0.7
  },

  //
  // Footstep plans
  //

  "plans":
  {
    "airbus_staircase":
    {
      "com_height": 0.78,
      "init_dsp_duration": 0.6,
      "single_support_duration": 1.4,
      "double_support_duration": 0.2,
      "final_dsp_duration": 0.6,
      "swing_height": 0.24,
      "landing_ratio": 0.1,
      "takeoff_ratio": 0.3,
      "contacts":
      [
        {
          "pose": { "translation": [-0.04,  0.09, 0.000] },
          "surface": "LeftFootCenter"
        },
        {
          "pause_after_swing": false,
          "pose": { "translation": [-0.04, -0.09, 0.000] },
          "surface": "RightFootCenter",
          "swing": { "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
        },
        {
          "pose": { "translation": [0.24,   0.09, 0.185] },
          "surface": "LeftFootCenter",
          "swing": { "takeoff_offset": [-0.02, 0.0, 0.0] }
        },
        {
          "pause_after_swing": false,
          "pose": { "translation": [0.24,  -0.09, 0.185] },
          "surface": "RightFootCenter",
          "swing": { "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
        },
        {
          "pose": { "translation": [0.48,   0.09, 0.370] },
          "surface": "LeftFootCenter",
          "swing": { "takeoff_offset": [-0.02, 0.0, 0.0] }
        },
        {
          "pause_after_swing": false,
          "pose": { "translation": [0.48,  -0.09, 0.370] },
          "surface": "RightFootCenter",
          "swing": { "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
        },
        {
          "pose": { "translation": [0.72,   0.09, 0.555] },
          "surface": "LeftFootCenter",
          "swing": { "takeoff_offset": [-0.02, 0.0, 0.0] }
        },
        {
          "pause_after_swing": false,
          "pose": { "translation": [0.72,  -0.09, 0.555] },
          "surface": "RightFootCenter",
          "swing": { "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
        },
        {
          "pose": { "translation": [0.96,   0.09, 0.740] },
          "surface": "LeftFootCenter",
          "swing": { "height": 0.2, "takeoff_offset": [-0.02, 0.0, 0.0] }
        },
        {
          "pause_after_swing": false,
          "pose": { "translation": [0.96,  -0.09, 0.740] },
          "surface": "RightFootCenter",
          "swing": { "height": 0.2, "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
        },
        { "pose": { "translation": [1.20,   0.09, 0.885] }, "surface": "LeftFootCenter"  },
        { "pose": { "translation": [1.20,  -0.09, 0.885] }, "surface": "RightFootCenter" }
      ],
      "hmpc":
      {
        "weights":
        {
          "jerk": 1.0,
          "vel": [10.0, 300.0],
          "zmp": 1000.0
        }
      }
    },
    "ashibumi":
    {
      "com_height": 0.78,
      "double_support_duration": 0.2,
      "single_support_duration": 0.8,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.035, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.035,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.035, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.035,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.035, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.035,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.035, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.035,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.035, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.035,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.035, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.035,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "backnforth_15cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.2,
      "single_support_duration": 0.8,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,   0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,   0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.15, -0.09, 0.0] }, "ref_vel": [0.15,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.3,   0.09, 0.0] }, "ref_vel": [0.15,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.45, -0.09, 0.0] }, "ref_vel": [0.0,   0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.45,  0.09, 0.0] }, "ref_vel": [0.0,   0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.3,  -0.09, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.15,  0.09, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,   0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,   0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "backnforth_20cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.25,
      "single_support_duration": 0.8,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.2,  0.09, 0.0] }, "ref_vel": [0.2,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.4, -0.09, 0.0] }, "ref_vel": [0.2,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.6,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.6, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.4,  0.09, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.2, -0.09, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" }
      ]
    },
    "backnforth_25cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.3,
      "single_support_duration": 0.8,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.25, -0.09, 0.0] }, "ref_vel": [0.2,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.5,   0.09, 0.0] }, "ref_vel": [0.2,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.75, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.75,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.5,  -0.09, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.25,  0.09, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "kajita_2010":
    {
      "com_height": 0.78,
      "double_support_duration": 0.2,
      "single_support_duration": 0.8,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.2,   -0.09, 0.0] }, "ref_vel": [0.12, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.45,   0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.725, -0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.975,  0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [1.25,  -0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [1.5,    0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [1.775, -0.09, 0.0] }, "ref_vel": [0.12, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [1.975,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [1.975, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" }
      ]
    },
    "koopa_30cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.2,
      "single_support_duration": 0.8,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.3, -0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.3,  0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.6, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.6,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "koopa_40cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.2,
      "single_support_duration": 0.8,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.4, -0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.4,  0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.8, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.8,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "koopa_50cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.2,
      "single_support_duration": 0.8,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.5, -0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.5,  0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [1.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [1.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "straight_15cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.1,
      "single_support_duration": 0.7,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.15, -0.09, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.3,   0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.45, -0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.6,   0.09, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.75, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.75,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "straight_20cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.1,
      "single_support_duration": 0.7,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.2, -0.09, 0.0] }, "ref_vel": [0.1, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.4,  0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.6, -0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.8,  0.09, 0.0] }, "ref_vel": [0.1, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [1.0, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [1.0,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "straight_25cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.1,
      "single_support_duration": 0.7,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.2, -0.09, 0.0] }, "ref_vel": [0.12, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.45, 0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.7, -0.09, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.95, 0.09, 0.0] }, "ref_vel": [0.12, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [1.15,-0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [1.15, 0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "straight_30cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.2,
      "single_support_duration": 0.8,
      "swing_height": 0.05,
      "contacts":
      [
        { "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.2, -0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.5,  0.09, 0.0] }, "ref_vel": [0.30, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.8, -0.09, 0.0] }, "ref_vel": [0.30, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [1.1,  0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [1.3, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [1.3,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "straight_35cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.2,
      "single_support_duration": 0.8,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.2,  -0.09, 0.0] }, "ref_vel": [0.17, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.55,  0.09, 0.0] }, "ref_vel": [0.35, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.9,  -0.09, 0.0] }, "ref_vel": [0.35, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [1.25,  0.09, 0.0] }, "ref_vel": [0.17, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [1.45, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [1.45,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "straight_40cm_steps":
    {
      "com_height": 0.78,
      "double_support_duration": 0.2,
      "single_support_duration": 0.7,
      "swing_height": 0.07,
      "contacts":
      [
        { "pose": { "translation": [0.0, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.2, -0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.6,  0.09, 0.0] }, "ref_vel": [0.4, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [1.0, -0.09, 0.0] }, "ref_vel": [0.4, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [1.4,  0.09, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [1.6, -0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [1.6,  0.09, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    },
    "walk_45cm":
    {
      "com_height": 0.78,
      "double_support_duration": 0.1,
      "single_support_duration": 0.7,
      "swing_height": 0.04,
      "contacts":
      [
        { "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.15, -0.09, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.3,   0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
        { "pose": { "translation": [0.45, -0.09, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
        { "pose": { "translation": [0.45,  0.09, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "LeftFootCenter" }
      ]
    }
  }
}
