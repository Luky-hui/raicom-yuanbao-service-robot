#!/usr/bin/env bash
set -euo pipefail

# Reset service venue textures to the clean baseline before official 3D event spawning.
# This prevents old texture-based fire/extinguisher events from stacking with official Gazebo models.
for venue in shenzhen guangzhou beijing shanghai jilin; do
  current=/home/yuanbao/.gazebo/models/rei_2025raicom/${venue}/meshes/${venue}.png
  baseline=${current}.bak_codex_20260622_235943
  if [[ -f ${baseline} ]]; then
    cp ${baseline} ${current}
  fi
done

cd /home/yuanbao/Downloads/zhangaiwu
exec /home/yuanbao/Downloads/zhangaiwu/reinovo_bobac3_sim
