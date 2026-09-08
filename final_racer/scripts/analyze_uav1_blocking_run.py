#!/usr/bin/env python3
"""Validate receive-before-execution from runtime events and plot a completed run."""
import json,re,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
suite=Path(sys.argv[1]).resolve()
case=suite/'formal_300s'
d=json.loads((case/'warehouse_full_distributed_result.json').read_text())
m=d['metrics']; comm=d['communication']; stats=comm['statistics']
received={}; waiting={}; enabled=set(); executed=set(); violations=[]; final_states={}; initial_publications=0
for line in (case/'warehouse_full_distributed_launch.log').open(errors='replace'):
    if 'Initial spatial partition epoch=' in line: initial_publications+=1
    match=re.search(r'RACER_BLOCKING_INITIAL_ENABLED uav=(\d+)',line)
    if match: enabled.add(int(match[1]))
    match=re.search(r'RACER_BLOCKING_INITIAL_WAIT uav=(\d+) time_s=([\d.]+)',line)
    if match: waiting[int(match[1])]=float(match[2])
    match=re.search(r'RACER_BLOCKING_INITIAL_RECEIVED uav=(\d+) time_s=([\d.]+)',line)
    if match: received.setdefault(int(match[1]),float(match[2]))
    match=re.search(r'Drone (\d+) from (\w+) to (\w+)',line)
    if match:
        u=int(match[1]); state=match[3];final_states[u]=state
        if state in ('PLAN_TRAJ','PUB_TRAJ','EXEC_TRAJ') and u not in received:
            violations.append(line.strip())
        if state=='EXEC_TRAJ': executed.add(u)
all_ids=set(range(1,11)); missing=all_ids-set(received)
checks={
 'elapsed_300s':299.9<=m['elapsed']<=301,
 'stop_reason_duration':m['stop_reason']=='duration',
 'all_10_blocking_enabled':enabled==all_ids,
 'all_10_entered_wait':set(waiting)==all_ids,
 'no_execution_before_assignment':not violations,
 'unassigned_remained_idle':all(final_states.get(u) in ('IDLE','IDL') for u in missing),
 'sionna_distributed':comm['mode']=='sionna' and comm['network_topology']=='distributed',
 'sionna_exact_samples_present':comm.get('exact_link_samples',0)>0,
 'no_perfect_startup_bypass':not stats.get('initial_assignment_perfect_delivery_enabled',False) and stats.get('initial_assignment_perfect_forwarded_packets',0)==0,
 'no_bs_traffic':all(stats.get(k,0)==0 for k in ('bs_uplink_attempted_packets','bs_downlink_attempted_packets','bs_control_attempted_packets')),
 'all_10_map_sources':m['mapping_coverage_joint_counts']['bitmap_sources_received']==10,
 'isaac_exit_ok':d['acceptance']['isaac_exit_ok'],
 'no_process_crashes':d['algorithm_evidence']['process_crashes']==0,
}
summary={'experiment_integrity_passed':all(checks.values()),'checks':checks,
 'elapsed_s':m['elapsed'],'union_coverage':m['mapping_coverage_joint'],
 'initial_partition_created_count':initial_publications,
 'assignment_received_time_s':received,'executed_uavs':sorted(executed),
 'never_received_uavs':sorted(missing),'final_fsm_states':final_states,
 'path_lengths_m':m['path_lengths'],'collision_events':m['collision_events'],
 'physics_contact_events':m['physics_contact_events'],
 'minimum_obstacle_clearance_m':m['min_obstacle_clearance'],
 'physical_delivery_ratio':stats['delivered_packets']/max(1,stats['attempted_packets']),
 'runner_passed':d['passed'],'runner_acceptance':d['acceptance'],
 'violations':violations,'source_result':str(case/'warehouse_full_distributed_result.json')}
(suite/'blocking_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'axes.spines.top':False,'axes.spines.right':False,'font.size':11})
fig,axes=plt.subplots(2,2,figsize=(14,10),layout='constrained')
h=m['mapping_coverage_joint_history']
axes[0,0].plot([v['time_s'] for v in h],[100*v['ratio'] for v in h],lw=2.5,color='#d97824')
axes[0,0].set(title=f"真实并集覆盖率：{100*m['mapping_coverage_joint']:.2f}%",xlabel='仿真时间（秒）',ylabel='覆盖率（%）',xlim=(0,300),ylim=(0,100));axes[0,0].grid(alpha=.2)
for u in range(1,11):
    t=received.get(u,300)
    axes[0,1].barh(u,t,color='#bcc5ce')
    if u in received:
        axes[0,1].barh(u,300-t,left=t,color='#2887a8')
    axes[0,1].text(303,u,f'{t:.2f}s 收到' if u in received else '未收到',va='center',fontsize=9)
axes[0,1].set(title='初始分配接收时刻｜灰：未获分配，蓝：收到后',xlabel='仿真时间（秒）',ylabel='UAV 编号',yticks=range(1,11),xlim=(0,385));axes[0,1].invert_yaxis()
colors=plt.get_cmap('tab10').colors
traj=np.array([v['positions'] for v in m['trajectory_history']])
for i in range(10):
    axes[1,0].plot(traj[:,i,0],traj[:,i,1],color=colors[i],lw=1.2,label=f'UAV {i+1}')
    axes[1,0].scatter(*traj[0,i,:2],color=colors[i],marker='*',s=65)
axes[1,0].set(title='俯视轨迹（星号为起点）',xlabel='X（米）',ylabel='Y（米）');axes[1,0].set_aspect('equal',adjustable='datalim');axes[1,0].grid(alpha=.2)
axes[1,1].bar(range(1,11),m['path_lengths'],color=[colors[i] if i+1 in received else '#bcc5ce' for i in range(10)])
axes[1,1].set(title=f"各机航程｜{len(received)}/10 收到任务，{len(executed)}/10 执行轨迹",xlabel='UAV 编号',ylabel='航程（米）',xticks=range(1,11));axes[1,1].grid(axis='y',alpha=.2)
if max(m['path_lengths']) == 0:
    axes[1,1].set_ylim(0,1)
    for u in range(1,11):axes[1,1].text(u,0.02,'0',ha='center',va='bottom')
fig.suptitle('纯分布式 Sionna｜UAV1 阻塞式初始任务分配｜300 秒\n10 UAV · 5 起飞点 · 100 MHz · 23 dBm · seed 42',fontsize=17)
fig.supxlabel(f"未收到初始任务则保持 idle，无等待超时；收到后执行剩余实验时间。\n接收前执行违规：{len(violations)}；碰撞计数：{m['collision_events']}；通用 runner passed：{d['passed']}。",fontsize=10)
for ext in ('png','pdf'):fig.savefig(suite/f'blocking_results.{ext}',dpi=160)
print(json.dumps(summary,indent=2))
if not all(checks.values()):sys.exit(1)
