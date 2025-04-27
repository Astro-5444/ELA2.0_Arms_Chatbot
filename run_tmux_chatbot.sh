#!/bin/bash

# Start new tmux session in detached mode
tmux new-session -d -s chatbot 'python3 /home/ela2/ELA2.0_Arms_Chatbot/src/Chatbot/ela2_ears.py'

# Split horizontally
tmux split-window -h -t chatbot 'python3 /home/ela2/ELA2.0_Arms_Chatbot/src/Chatbot/Chatbot.py'

# Split vertically from pane 0
tmux select-pane -t 0
tmux split-window -v -t chatbot 'python3 /home/ela2/ELA2.0_Arms_Chatbot/src/Chatbot/ela2_mouth.py'

# Split vertically from pane 1
#tmux select-pane -t 1
tmux split-window -v -t chatbot './set-default-sink.sh'

tmux select-layout -t chatbot tiled
tmux attach-session -t chatbot

