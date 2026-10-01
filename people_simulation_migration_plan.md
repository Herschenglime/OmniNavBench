# OmniNavBench People Simulation & Human Following Migration Plan

## 1. Executive Summary & Context

In NVIDIA Isaac Sim 6.0 and 6.1 (Omniverse Kit 109 SDK), the legacy pedestrian simulation extension `omni.anim.people` was removed from the default distribution. In OmniNavBench, several modules previously depended directly on `omni.anim.people`:
1. **Procedural Animation Graph Generation**: `omni.anim.people.scripts.custom_command.populate_anim_graph`
2. **Dynamic Obstacle Script Attachment**: `omni.anim.people.python_ext.add_dynamic_obstacle_behavior_script`
3. **Agent Registration Events**: `omni.anim.people.settings.AgentEvent`
4. **Behavior Scripts & Route Control**: `character_behavior.py`, `interactable_object_helper.py`, and command text files.

To prioritize stabilizing the core navigation benchmark (static architectural scenes, mobile robots like `carter_v1`, LiDAR/RGB-D sensor pipelines, and policy evaluation), people simulation is **safely guarded and disabled in Phase 1**.

This document outlines the detailed roadmap for porting human-following and dynamic pedestrian simulation to the modern Isaac Sim 6.1 architecture.

---

## 2. Isaac Sim 6.1 Modern Architecture Replacement

NVIDIA has split character and crowd simulation into two decoupled subsystems:

```mermaid
flowchart TD
    subgraph Legacy [Isaac Sim <= 5.0 (Legacy)]
        OAP["omni.anim.people"]
        OAP --> PAG["populate_anim_graph.py"]
        OAP --> DOS["dynamic_obstacle.py (Python Script)"]
        OAP --> OAP_EVT["AgentEvent (Carb)"]
    end

    subgraph Modern [Isaac Sim 6.1 (Modern)]
        IRA["isaacsim.replicator.agent (IRA)"]
        BH_CORE["omni.anim.behavior.core"]
        BT_CORE["omni.behavior.tree.core"]
        NAV_CORE["omni.anim.navigation.core"]
        
        IRA --> |Spawns & Configures| CHAR["USD SkelRoot Characters"]
        IRA --> |Dispatches| IRA_EVT["IRAEvents (ON_AGENT_REGISTERED)"]
        BH_CORE --> |Executes| MOTION["Motion Libraries (HumanMotionLibrary.usd)"]
        BH_CORE --> |C++ Collision Avoidance| NAV_CORE
        BT_CORE --> |Controls Behavior| TREE["Behavior Trees (.obt files)"]
    end

    Legacy -.-> |Migrate To| Modern
```

### Architectural Mapping

| Component | Legacy `omni.anim.people` | Modern Isaac Sim 6.1 Stack |
| :--- | :--- | :--- |
| **Character Spawning** | Procedural `Biped_Setup.usd` | USD SkelRoot + `ApplyIRACharacterAPICommand` / `ApplyBehaviorTreeAPICommand` |
| **Animation Graph** | Scripted `populate_anim_graph()` | USD-embedded AnimationGraph, `ApplyAnimationGraphAPICommand`, or Motion Library |
| **Dynamic Obstacles** | Injected `dynamic_obstacle.py` script | Native PhysX collider detection via `omni.anim.behavior.core` (`enableAutoAvoidance = true`) |
| **Behavior Engine** | Command files (`.txt` GoTo/Idle) | Behavior Trees (`.obt` YAML/JSON) or `IBehaviorAgent` C++ interface |
| **Agent Events** | `AgentEvent.AgentRegistered` | `isaacsim.replicator.agent.core.events.IRAEvents.ON_AGENT_REGISTERED_EVENT` |

---

## 3. Migration Roadmap (Phase-by-Phase)

### Phase A: Asset & Motion Library Setup
1. **Replace Legacy Assets**:
   - Retire legacy `/Isaac/People/Characters/Biped_Setup.usd` and `biped_demo_meters.usd`.
   - Update `AssetPaths.default_biped_asset_path()` to point to standard Isaac Sim 6.1 character assets equipped with SkelRoot and modern skinning.
2. **Motion Library Configuration**:
   - Configure IRA's default motion library asset (`HumanMotionLibrary.usd` or local mirror in `/home/robopi/data/Assets`).

### Phase B: Animation & Character Binding
1. **Schema-Based Binding**:
   - Replace the legacy call to `populate_anim_graph()` in `OmniNavExt/envset/stage_util.py::setup_biped_prim()` with modern binding:
     ```python
     omni.kit.commands.execute("ApplyAnimationGraphAPICommand", paths=[skel_root_path], animation_graph_path=anim_graph_path)
     ```
   - For behavior-tree driven characters, use `ApplyBehaviorTreeAPICommand`.

### Phase C: Agent Event & Lifecycle Migration
1. **Agent Manager Update**:
   - Migrate `OmniNavExt/envset/agent_manager.py` and `runtime_hooks.py` to listen to `IRAEvents.ON_AGENT_REGISTERED_EVENT` from `isaacsim.replicator.agent.core.events`.
2. **Route Injection**:
   - Replace the legacy text-based command queue (`GoToCommand`, `TimingCommand`) with the `IBehaviorAgent` API:
     ```python
     import omni.anim.behavior.core as bh_core
     bh = bh_core.acquire_interface()
     agent = bh.get_agent(character_prim_path)
     agent.move_to_location(target_pos, speed=1.2)
     ```

### Phase D: Dynamic Obstacle Avoidance
1. **PhysX Collision Registration**:
   - In `OmniNavExt/envset/incident_bridge.py`, eliminate the python script injection.
   - Ensure dynamic obstacles (such as mobile robots or falling objects) have `UsdPhysics.CollisionAPI` applied so that `omni.anim.behavior.core` queries them automatically during local path planning.

### Phase E: Human-Following Evaluation Protocol
1. **Task Adapter Validation**:
   - Validate `bench/tasks/human_following.py` against the `IBehaviorAgent` ground-truth trajectory coordinates.
   - Run end-to-end evaluation with a moving pedestrian target and verify metric reporting (Success Rate, SPL, Detour Index).

---

## 4. Current State & Temporary Guard Status

While the full migration is deferred:
- `populate_anim_graph` is wrapped with a no-op fallback in `stage_util.py`.
- `add_dynamic_obstacle_behavior_script` is wrapped with a no-op fallback in `incident_bridge.py`.
- Event subscriptions in `agent_manager.py` and `runtime_hooks.py` gracefully handle the absence of `omni.anim.people`.
- This ensures static-scene navigation benchmarks (e.g. `grscenes_test_ep1.json`) run without unhandled import errors or simulation crashes.
