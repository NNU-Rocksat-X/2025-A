## Auto SSH Login
SSH Keys can be used so you are not asked to type in the password each time.

To setup auto login on a windows pc run the following commands in Powershell:

The first step creates a key for your pc.
**This step only needs to be complete once**
```
ssh-keygen
```
After entering the command, you will be asked for a file, and a passphrase for the key. You can leave these blank by pressing enter for both prompts.

Next enter the command to send your ssh-key to the remote device:
```
type $env:USERPROFILE\.ssh\id_rsa.pub | ssh <USER@IP-ADDRESS> "cat >> .ssh/authorized_keys"
```

You can now remote SSH without being prompted for a password.


