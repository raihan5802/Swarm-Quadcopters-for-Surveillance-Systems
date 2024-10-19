#!/bin/bash

# Read the password from password.txt
PASSWORD=$(<password.txt)

# Read the hosts from hosts.txt
HOSTS=$(<hosts.txt)

# Loop to read commands and execute them on all hosts
while read -r COMMAND; do
    # Use GNU Parallel with sshpass to run the command on each host
    parallel -u -k sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null {} "source /home/swarm/swarm/bin/activate && $COMMAND" ::: $HOSTS
done

