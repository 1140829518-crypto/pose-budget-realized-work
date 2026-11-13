# CERES_VALIDATION_NEW_20261111 最终报告

## 最终状态

工程目标已真实完成：独立worktree、13项确认补丁恢复、快照导出接入、Ceres 2.2.0独立构建、双后端四轮回放、5个独立诊断Run、预实验、冻结清单及50条正式记录均已落盘。正式记录50成功、0失败；一次非正式诊断捕获因导出器未链接产生0快照，已永久保留并从有效输入中排除。

## 核心数据

- 5个独立Run，759个真实PoseOptimization调用，3,036个冻结四轮快照。
- 50条Run级正式记录，7,590条调用结果，30,360条轮次结果。
- 3,795对同输入g2o—Ceres调用；初始目标函数最大绝对差0。
- 正式失败记录0；无NaN/Inf、重复主键或调用/轮次遗漏。
- Linux和Windows F盘：5个诊断Run归档、50个正式记录归档均通过SHA256一致性检查。

## 科学边界

本批次是 `CHAINED_OFFLINE_REPLAY_V1` 离线统一问题回放，不是原在线四轮语义的逐位复刻，不是新的SLAM算法，也不是完整SLAM端到端复现。它不生成ATE/RPE，不并入原历史60次或扩展60次数据。统计独立单位为5个Run，调用和轮次不作独立样本。结果支持BSRW最小测量契约可在g2o和Ceres上实现，并显示BRR、触顶率、E_W具有互补信息；不支持后端等效、Ceres或g2o普遍更优、B5普适最优或跨硬件普适阈值。

## 论文候选稿

已基于冻结横向比较稿生成非覆盖候选：`paper_candidate/PAPER2_CES_THIRD_REVISION_CANDIDATE.docx`、`.pdf`、`.tex`和`.html`。新增最小测量契约、独立后端实验设计、正式结果、边界和Ceres官方软件引用。PDF为A4、20页，无空白页；已逐页文本检查并重点目视检查首页、新增表图、讨论、结论与参考文献页。原冻结Word/PDF/LaTeX未覆盖。

## 主项目保护

- 独立分支：`paper2/ceres-validation-new-20261111`。
- 独立worktree：`<LOCAL_CERES_VALIDATION_ROOT>/`。
- 主项目未执行commit、push、reset或clean。
- 主项目已跟踪未暂存diff SHA256在正式执行前后均为`c9ef8918097ce7679397d45b43ba9dc0a3f29ac8dbad89c5b96b5d7d297c8a0a`；暂存diff为空哈希`e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`。

该工程具备进入作者审阅和论文第三轮定稿阶段的条件；在作者确认前，候选稿不替代冻结投稿基线。
