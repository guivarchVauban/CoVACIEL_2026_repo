#!/bin/bash

echo "🚀 Mode Fainéant Activé..."

# 1. On va dans le bon dossier
cd ~/ros2_ws

# 2. On compile UNIQUEMENT nav_tunnel (gain de temps)
echo "🔨 Compilation en cours..."
colcon build --packages-select nav_tunnel

# 3. Vérification : Si ça compile, on lance. Sinon, on arrête.
if [ $? -eq 0 ]; then
    echo "✅ Compilation réussie ! Lancement du robot..."
    source install/setup.bash
    ros2 run nav_tunnel nav_tunnel
else
    echo "❌ Erreur dans ton code ! Corrige avant de lancer."
    exit 1
fi
