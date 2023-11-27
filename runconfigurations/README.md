# Errors in Artemus
## Enter problem because of windows
When working on Windows to get the running configurations perfectly, 
sometimes errors are introduced because of the way windows works with enters.

```/bin/bash^M: bad interpreter: No such file or directory```

When one get's this error, it means that the windows enter type, shown by the ^M, does not play nicely with linux.
To solve this, run the following command in each runconfigurations directory.

``` sed -i -e 's/\r$//' *```

More info on it can be found [here](https://askubuntu.com/questions/304999/not-able-to-execute-a-sh-file-bin-bashm-bad-interpreter).


## Resources problem.
If your job was killed, it was probably because it used to much RAM.
To check it out, execute 
``` qstat -xf 1234567 ```

Other commands are
```
qstat -u abcd1234       show status of abcd1234’s jobs
qdel 1234567            delete job 1234567 from queue
qstat                   show status of all jobs
qstat -f 1234567        show detailed stats for job 1234567
qstat -xf 1234567       show detailed stats for job 1234567, even after it has finished
```