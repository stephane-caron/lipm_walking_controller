{
  "initial_plan": "custom_forward",
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
      "comanoid_airbus_staircase": // https://youtu.be/vFCFKAunsYM
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
      "walk_backward_75cm":
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
      "walk_forward_100cm":
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
      "walk_backward_75cm":
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
      "walk_forward_100cm":
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
      "comanoid_airbus_staircase":
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
      "walk_backward_75cm":
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
      "walk_forward_100cm":
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
