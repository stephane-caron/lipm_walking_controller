{
  "initial_plan": "warmup",
  "mpc":
  {
    "weights":
    {
      "jerk": 1.0,
      "vel": [10.0, 100.0],
      "zmp": 1000.0
    }
  },
  "stabilizer":
  {
    "fdqp_weights":
    {
      "net_wrench": 10000.0,
      "ankle_torque": 100.0,
      "pressure": 1.0
    },
    "tasks":
    {
      "com":
      {
        "stiffness": [1000.0, 1000.0, 100.0],
        "weight": 1000.0
      },
      "contact":
      {
        "damping": 300.0,
        "stiffness": 1.0,
        "weight": 10000.0
      },
      "swing_foot":
      {
        "stiffness": 2000.0,
        "weight": 500.0
      }
    },
    "vdc":
    {
      "frequency": 1.0,
      "stiffness": 1000.0
    },
    "zmpcc":
    {
      "integrator_leak_rate": 0.1
    }
  },
  "tasks":
  {
    "pelvis":
    {
      "stiffness": 10.0,
      "weight": 100.0
    },
    "posture":
    {
      "stiffness": 1.0,
      "weight": 10.0
    },
    "torso":
    {
      "stiffness": 10.0,
      "weight": 100.0
    }
  },
  "robot_models":
  {
    "hrp4":
    {
      "admittance":
      {
        "com": [0.0, 0.0],
        "cop": [0.01, 0.01],
        "dfz": 0.0001,
        "dfz_damping": 0.0
      },
      "com":
      {
        "active_joints": [
          "Root",
          "R_HIP_Y", "R_HIP_R", "R_HIP_P", "R_KNEE_P", "R_ANKLE_P", "R_ANKLE_R",
          "L_HIP_Y", "L_HIP_R", "L_HIP_P", "L_KNEE_P", "L_ANKLE_P", "L_ANKLE_R"
        ],
        "height": 0.78,
        "max_height": 0.85,
        "min_height": 0.55
      },
      "dcm_tracking":
      {
        "gains":
        {
          "prop": 5.0,
          "integral": 10.0,
          "deriv": 0.0
        },
        "derivator_time_constant": 1.0,
        "integrator_time_constant": 10.0
      },
      "sole":
      {
        "half_length": 0.112,
        "half_width": 0.065,
        "friction": 0.7
      },
      "torso":
      {
        "max_pitch": 0.4,
        "min_pitch": -0.1,
        "name": "torso",
        "pitch": 0.1
      }
    },
    "hrp2_drc":
    {
      "admittance":
      {
        "com": [0.0, 0.0],
        "cop": [0.01, 0.01],
        "dfz": 0.0001,
        "dfz_damping": 0.0
      },
      "com":
      {
        "active_joints": [
          "Root",
          "RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2", "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
          "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2", "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5"
        ],
        "height": 0.87,
        "max_height": 0.92,
        "min_height": 0.6
      },
      "dcm_tracking":
      {
        "gains":
        {
          "prop": 3.0,
          "integral": 20.0,
          "deriv": 0.5
        },
        "derivator_time_constant": 5.0,
        "integrator_time_constant": 30.0
      },
      "sole":
      {
        "half_length": 0.108,
        "half_width": 0.07,
        "friction": 0.7
      },
      "torso":
      {
        "max_pitch": 0.4,
        "min_pitch": -0.1,
        "name": "CHEST_LINK1",
        "pitch": 0.0
      }
    },
    "jvrc1":
    {
      "admittance":
      {
        "com": [0.0, 0.0],
        "cop": [0.01, 0.01],
        "dfz": 0.0001,
        "dfz_damping": 0.0
      },
      "com":
      {
        "active_joints": [
          "Root",
          "R_HIP_Y", "R_HIP_R", "R_HIP_P", "R_KNEE", "R_ANKLE_P", "R_ANKLE_R",
          "L_HIP_Y", "L_HIP_R", "L_HIP_P", "L_KNEE", "L_ANKLE_P", "L_ANKLE_R"
        ],
        "height": 0.85,
        "max_height": 0.90,
        "min_height": 0.65
      },
      "dcm_tracking":
      {
        "gains":
        {
          "prop": 5.0,
          "integral": 10.0,
          "deriv": 0.0
        },
        "derivator_time_constant": 1.0,
        "integrator_time_constant": 10.0
      },
      "sole":
      {
        "half_length": 0.11,
        "half_width": 0.05,
        "friction": 0.7
      },
      "torso":
      {
        "max_pitch": 0.4,
        "min_pitch": -0.1,
        "name": "WAIST_R_S",
        "pitch": 0.0
      }
    }
  },
  "plans":
  {
    "hrp4":
    {
      "ashibumi": // stepping in place
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" }
        ]
      },
      "ashibumi_fast":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter" }
        ]
      },
      "backward_15cm_steps":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.05,
        "contacts":
        [
          { "pose": { "translation": [0.0,   -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,    0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-0.15, -0.09, 0.0] }, "ref_vel": [-0.075, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-0.3,   0.09, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-0.45, -0.09, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-0.6,   0.09, 0.0] }, "ref_vel": [-0.075, 0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-0.75, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-0.75,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  }
        ]
      },
      "backward_20cm_steps":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.05,
        "contacts":
        [
          { "pose": { "translation": [0.0,  -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,   0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-0.2, -0.09, 0.0] }, "ref_vel": [-0.1, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-0.4,  0.09, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-0.6, -0.09, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-0.8,  0.09, 0.0] }, "ref_vel": [-0.1, 0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-1.0, -0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-1.0,  0.09, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  }
        ]
      },
      "comanoid_airbus_staircase": // 1st part of Sep 10, 2019 experiment video (on YouTube)
      {
        "init_dsp_duration": 0.6,
        "single_support_duration": 1.4,
        "double_support_duration": 0.2,
        "final_dsp_duration": 0.6,
        "swing_height": 0.24,
        "landing_duration": 0.1,
        "takeoff_duration": 0.42,
        "torso_pitch": 0.2,
        "contacts":
        [
          {
            "pose": { "translation": [-0.04,  0.09, 0.000] },
            "surface": "LeftFootCenter"
          },
          {
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
            "pose": { "translation": [0.96,  -0.09, 0.740] },
            "surface": "RightFootCenter",
            "swing": { "height": 0.2, "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
          },
          { "pose": { "translation": [1.20,   0.09, 0.885] }, "surface": "LeftFootCenter"  },
          { "pose": { "translation": [1.20,  -0.09, 0.885] }, "surface": "RightFootCenter" }
        ],
        "mpc":
        {
          "weights":
          {
            "jerk": 1.0,
            "vel": [10.0, 300.0],
            "zmp": 1000.0
          }
        }
      },
      "comanoid_walk_45cm":  // 2nd part of Sep 10, 2019 experiment video (on YouTube)
      {
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
      },
      "curved_15cm_steps":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": {"translation": [2.77078, -0.590245, 0], "rotation": [0.940452, 0.339926, 0, -0.339926, 0.940452, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [2.74776, -0.38652, 0], "rotation": [0.875393, 0.483412, 0, -0.483412, 0.875393, 0, 0, 0, 1] }, "surface": "LeftFootCenter" }, 
          { "pose": {"translation": [2.95499, -0.503226, 0 ], "rotation": [0.893052, 0.449954, 0, -0.449954, 0.893052, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [2.98469, -0.267551, 0 ], "rotation": [0.912718, 0.408589, 0, -0.408589, 0.912718, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [3.18044, -0.402869, 0 ], "rotation": [0.93489, 0.354938, 0, -0.354938, 0.93489, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [3.23814, -0.172172, 0 ], "rotation": [0.958776, 0.284164, 0, -0.284164, 0.958776, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [3.4113, -0.335461, 0 ], "rotation": [0.982527, 0.186121, 0, -0.186121, 0.982527, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [3.51403, -0.121684, 0 ], "rotation": [0.998708, 0.0508231, 0, -0.0508231, 0.998708, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [3.63427, -0.325807, 0 ], "rotation": [0.989986, -0.141166, 0, 0.141166, 0.989986, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [3.81101, -0.167986, 0 ], "rotation": [0.922807, -0.385262, 0, 0.385262, 0.922807, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [3.81955, -0.403709, 0 ], "rotation": [0.77646, -0.630166, 0, 0.630166, 0.77646, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [4.05162, -0.36087, 0 ], "rotation": [0.589049, -0.808097, 0, 0.808097, 0.589049, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [3.92777, -0.518618, 0 ], "rotation": [0.500388, -0.865801, 0, 0.865801, 0.500388, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [4.05162, -0.46087, 0 ], "rotation": [0.589049, -0.808097, 0, 0.808097, 0.589049, 0, 0, 0, 1] }, "surface": "LeftFootCenter" }
        ]
      },
      "curved_20cm_steps":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": {"translation": [0.0393791, -0.0897642, 0], "rotation": [0.999997, -0.00256075, 0, 0.00256075, 0.999997, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [0.0393791, 0.0897642, 0], "rotation": [0.999997, -0.00256075, 0, 0.00256075, 0.999997, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [0.120362, -0.108239, 0], "rotation": [0.981797, -0.189932, 0, 0.189932, 0.981797, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [0.351217, 0.0276016, 0], "rotation": [0.976051, -0.217542, 0, 0.217542, 0.976051, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [0.503607, -0.193613, 0], "rotation": [0.969694, -0.244323, 0, 0.244323, 0.969694, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [0.741152, -0.0704351, 0], "rotation": [0.963175, -0.268874, 0, 0.268874, 0.963175, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [0.883438, -0.29945, 0], "rotation": [0.956937, -0.290295, 0, 0.290295, 0.956937, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [1.12694, -0.187005, 0], "rotation": [0.951955, -0.306238, 0, 0.306238, 0.951955, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [1.2605, -0.420172, 0], "rotation": [0.949275, -0.314448, 0, 0.314448, 0.949275, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [1.50508, -0.311387, 0], "rotation": [0.950331, -0.311241, 0, 0.311241, 0.950331, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [1.64046, -0.543348, 0], "rotation": [0.95659, -0.291438, 0, 0.291438, 0.95659, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [1.87991, -0.423968, 0], "rotation": [0.968801, -0.247841, 0, 0.247841, 0.968801, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [2.03754, -0.64234, 0], "rotation": [0.985361, -0.170483, 0, 0.170483, 0.985361, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [2.25466, -0.486607, 0], "rotation": [0.998584, -0.0532061, 0, 0.0532061, 0.998584, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [2.45886, -0.661726, 0], "rotation": [0.994881, 0.101058, 0, -0.101058, 0.994881, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [2.62191, -0.448698, 0], "rotation": [0.964239, 0.265033, 0, -0.265033, 0.964239, 0, 0, 0, 1] }, "surface": "LeftFootCenter" },
          { "pose": {"translation": [2.77046, -0.590235, 0], "rotation": [0.94045, 0.339933, 0, -0.339933, 0.94045, 0, 0, 0, 1] }, "surface": "RightFootCenter" },
          { "pose": {"translation": [2.74776, -0.38652, 0], "rotation": [0.875393, 0.483412, 0, -0.483412, 0.875393, 0, 0, 0, 1] }, "surface": "LeftFootCenter" }
        ]
      },
      "custom_backward":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "step_length": 0.15,
        "swing_height": 0.05,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.09, 0.0] }, "surface": "LeftFootCenter"  }
        ]
      },
      "custom_forward":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "step_length": 0.2,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.09, 0.0] }, "surface": "LeftFootCenter"  }
        ]
      },
      "custom_lateral":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "step_length": 0.1,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.09, 0.0] }, "surface": "LeftFootCenter"  }
        ],
        "mpc":
        {
          "weights":
          {
            "jerk": 1.0,
            "vel": [10.0, 300.0],
            "zmp": 1000.0
          }
        }
      },
      "forward_15cm_steps":
      {
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
      "forward_20cm_steps":
      {
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
      "forward_25cm_steps":
      {
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
      "forward_30cm_steps":
      {
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
      "forward_35cm_steps":
      {
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
      "forward_40cm_steps":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
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
      "kajita_2010":
      {
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
      "lateral_left_10cm_steps":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.19, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0,  0.01, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.29, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0,  0.11, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.39, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0,  0.21, 0.0] }, "surface": "RightFootCenter" }
        ],
        "mpc":
        {
          "weights":
          {
            "jerk": 1.0,
            "vel": [10.0, 300.0],
            "zmp": 1000.0
          }
        }
      },
      "lateral_right_10cm_steps":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.09, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.19, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0, -0.01, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.29, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0, -0.11, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.39, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0, -0.21, 0.0] }, "surface": "LeftFootCenter" }
        ],
        "mpc":
        {
          "weights":
          {
            "jerk": 1.0,
            "vel": [10.0, 300.0],
            "zmp": 1000.0
          }
        }
      },
      "warmup":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter"  },
          { "pose": { "translation": [0.035, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.09, 0.0] }, "surface": "LeftFootCenter"  }
        ]
      }
    },
    "hrp2_drc":
    {
      "ashibumi": // stepping in place
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" }
        ]
      },
      "backward_15cm_steps":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.05,
        "contacts":
        [
          { "pose": { "translation": [0.0,   -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,    0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-0.15, -0.105, 0.0] }, "ref_vel": [-0.075, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-0.3,   0.105, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-0.45, -0.105, 0.0] }, "ref_vel": [-0.15, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-0.6,   0.105, 0.0] }, "ref_vel": [-0.075, 0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-0.75, -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-0.75,  0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  }
        ]
      },
      "backward_20cm_steps":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.05,
        "contacts":
        [
          { "pose": { "translation": [0.0,  -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,   0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-0.2, -0.105, 0.0] }, "ref_vel": [-0.1, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-0.4,  0.105, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-0.6, -0.105, 0.0] }, "ref_vel": [-0.2, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-0.8,  0.105, 0.0] }, "ref_vel": [-0.1, 0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [-1.0, -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [-1.0,  0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  }
        ]
      },
      "custom_backward":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "step_length": 0.15,
        "swing_height": 0.05,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter"  }
        ]
      },
      "custom_forward":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "step_length": 0.2,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter"  }
        ]
      },
      "custom_lateral":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "step_length": 0.1,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter"  }
        ],
        "mpc":
        {
          "weights":
          {
            "jerk": 1.0,
            "vel": [10.0, 300.0],
            "zmp": 1000.0
          }
        }
      },
      "forward_15cm_steps":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0,  -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,   0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.15, -0.105, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.3,   0.105, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.45, -0.105, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.6,   0.105, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.75, -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.75,  0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
        ]
      },
      "forward_20cm_steps":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.2, -0.105, 0.0] }, "ref_vel": [0.1, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.4,  0.105, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.6, -0.105, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.8,  0.105, 0.0] }, "ref_vel": [0.1, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [1.0, -0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [1.0,  0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" }
        ]
      },
      "step_40cm_forward":
      {
        "init_dsp_duration": 0.6,
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "final_dsp_duration": 0.6,
        "swing_height": 0.04,
        "torso_pitch": 0.0,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "ref_vel": [0.1,  0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [0.4, -0.105, 0.0] }, "ref_vel": [0.1,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.4,  0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  }
        ],
        "mpc":
        {
          "weights":
          {
            "jerk": 1.0,
            "vel": [100.0, 100.0],
            "zmp": 1000.0
          }
        }
      },
      "warmup":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.035, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.105, 0.0] }, "surface": "LeftFootCenter"  },
          { "pose": { "translation": [0.035, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.105, 0.0] }, "surface": "LeftFootCenter"  }
        ]
      }
    },
    "jvrc1":
    {
      "ashibumi": // stepping in place
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "surface": "LeftFootCenter" }
        ]
      },
      "comanoid_airbus_staircase": // 1st part of Sep 10, 2019 experiment video (on YouTube)
      {
        "com_height": 0.84,
        "init_dsp_duration": 0.6,
        "single_support_duration": 1.4,
        "double_support_duration": 0.2,
        "final_dsp_duration": 0.6,
        "swing_height": 0.24,
        "landing_duration": 0.1,
        "takeoff_duration": 0.42,
        "torso_pitch": 0.2,
        "contacts":
        [
          {
            "pose": { "translation": [-0.04,  0.09, 0.000] },
            "surface": "LeftFootCenter"
          },
          {
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
            "pose": { "translation": [0.96,  -0.09, 0.740] },
            "surface": "RightFootCenter",
            "swing": { "height": 0.2, "takeoff_offset": [-0.03, 0.0, 0.0], "takeoff_pitch": 0.6 }
          },
          { "pose": { "translation": [1.20,   0.09, 0.885] }, "surface": "LeftFootCenter"  },
          { "pose": { "translation": [1.20,  -0.09, 0.885] }, "surface": "RightFootCenter" }
        ],
        "mpc":
        {
          "weights":
          {
            "jerk": 1.0,
            "vel": [10.0, 300.0],
            "zmp": 1000.0
          }
        }
      },
      "comanoid_walk_45cm":  // 2nd part of Sep 10, 2019 experiment video (on YouTube)
      {
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
      },
      "custom_backward":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "step_length": 0.15,
        "swing_height": 0.05,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.09, 0.0] }, "surface": "LeftFootCenter"  }
        ]
      },
      "custom_forward":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "step_length": 0.2,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.09, 0.0] }, "surface": "LeftFootCenter"  }
        ]
      },
      "custom_lateral":
      {
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "step_length": 0.1,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.09, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.09, 0.0] }, "surface": "LeftFootCenter"  }
        ],
        "mpc":
        {
          "weights":
          {
            "jerk": 1.0,
            "vel": [10.0, 300.0],
            "zmp": 1000.0
          }
        }
      },
      "forward_15cm_steps":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0,  -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,   0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.15, -0.105, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.3,   0.105, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.45, -0.105, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.6,   0.105, 0.0] }, "ref_vel": [0.07, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.75, -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.75,  0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
        ]
      },
      "forward_20cm_steps":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.2, -0.105, 0.0] }, "ref_vel": [0.12, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.4,  0.105, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.6, -0.105, 0.0] }, "ref_vel": [0.25, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.8,  0.105, 0.0] }, "ref_vel": [0.12, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [1.0, -0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [1.0,  0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" }
        ]
      },
      "forward_25cm_steps":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.2, -0.105, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.45, 0.105, 0.0] }, "ref_vel": [0.30, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.7, -0.105, 0.0] }, "ref_vel": [0.30, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.95, 0.105, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [1.15,-0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [1.15, 0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
        ]
      },
      "forward_30cm_steps":
      {
        "com_height": 0.83,
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.05,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.2, -0.105, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.5,  0.105, 0.0] }, "ref_vel": [0.30, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.8, -0.105, 0.0] }, "ref_vel": [0.30, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [1.1,  0.105, 0.0] }, "ref_vel": [0.15, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [1.3, -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [1.3,  0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
        ]
      },
      "forward_35cm_steps":
      {
        "com_height": 0.83,
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.0,  -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,   0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.2,  -0.105, 0.0] }, "ref_vel": [0.17, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.55,  0.105, 0.0] }, "ref_vel": [0.35, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.9,  -0.105, 0.0] }, "ref_vel": [0.35, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [1.25,  0.105, 0.0] }, "ref_vel": [0.17, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [1.45, -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [1.45,  0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter" }
        ]
      },
      "forward_40cm_steps":
      {
        "com_height": 0.83,
        "double_support_duration": 0.2,
        "single_support_duration": 0.8,
        "swing_height": 0.07,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [0.2, -0.105, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.6,  0.105, 0.0] }, "ref_vel": [0.4, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [1.0, -0.105, 0.0] }, "ref_vel": [0.4, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [1.4,  0.105, 0.0] }, "ref_vel": [0.2, 0.0, 0.0], "surface": "LeftFootCenter" },
          { "pose": { "translation": [1.6, -0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [1.6,  0.105, 0.0] }, "ref_vel": [0.0, 0.0, 0.0], "surface": "LeftFootCenter" }
        ]
      },
      "step_40cm_forward":
      {
        "com_height": 0.83,
        "init_dsp_duration": 0.6,
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "final_dsp_duration": 0.6,
        "swing_height": 0.04,
        "torso_pitch": 0.0,
        "contacts":
        [
          { "pose": { "translation": [0.0, -0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.0,  0.105, 0.0] }, "ref_vel": [0.1,  0.0, 0.0], "surface": "LeftFootCenter"  },
          { "pose": { "translation": [0.4, -0.105, 0.0] }, "ref_vel": [0.1,  0.0, 0.0], "surface": "RightFootCenter" },
          { "pose": { "translation": [0.4,  0.105, 0.0] }, "ref_vel": [0.0,  0.0, 0.0], "surface": "LeftFootCenter"  }
        ],
        "mpc":
        {
          "weights":
          {
            "jerk": 1.0,
            "vel": [100.0, 100.0],
            "zmp": 1000.0
          }
        }
      },
      "warmup":
      {
        "double_support_duration": 0.1,
        "single_support_duration": 0.7,
        "swing_height": 0.04,
        "contacts":
        [
          { "pose": { "translation": [0.035, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.105, 0.0] }, "surface": "LeftFootCenter"  },
          { "pose": { "translation": [0.035, -0.105, 0.0] }, "surface": "RightFootCenter" },
          { "pose": { "translation": [0.035,  0.105, 0.0] }, "surface": "LeftFootCenter"  }
        ]
      }
    }
  },

  //
  // Tasks
  //

  "constraints":
  [
    {
      "type": "contact"
    },
    {
      "type": "kinematics", // KinematicsConstraint, i.e. joint limits
      "damper": [           // see equation (3.6) in Joris Vaillant's thesis
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
  // Finite state machine
  // 

  "Managed": false,
  "StatesFiles": [],
  "StatesLibraries": ["@MC_RTC_LIBDIR@/mc_controller/lipm_walking_controller/states"],
  "StepByStep": false,
  "configs": {},
  "init": "Initial",
  "states": {},
  "transitions":
  [
    ["Initial", "Standing", "Standing"],
    ["Standing", "DoubleSupport", "DoubleSupport"],
    ["DoubleSupport", "SingleSupport", "SingleSupport"],
    ["DoubleSupport", "Standing", "Standing"],
    ["SingleSupport", "DoubleSupport", "DoubleSupport"]
  ],

  // Set realRobot's joint configuration from encoder readings
  "RunObservers": ["Encoder"],
  "UpdateObservers": ["Encoder"]
}
