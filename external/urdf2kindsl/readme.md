
This is the repository of the URDF2KinDSL tool. The URDF2KinDSL is a converter
from the [URDF][urdf-web] to the [Kinematics-DSL][kindsl-web] robot model
formats.

# Requirements

* Python (2.7 or 3, both _should_ work)
* NumPy for Python

If in doubt, use Python 3

# Usage
## Command line
From the root of the repository, run:

```
./urdf2kindsl.py --help
```

Refer to the command-line help for usage and options.

## From Python code
Refer to the main function `urdf2kindsl.cmdline.main()` for an example of how
to use the classes in the package to perform the conversion.

# Testing
Tests are mainly regression tests for developers of the tool. However, to run
the test suite, issue the following command from the root of the repository:

```
python[3] -m urdf2kindsl.test.testing
```


# Limitations

TODO...


# Copyright notice

Copyright © 2018-2019 Marco Frigerio

Released under the BSD 2-clause license. See the `LICENSE` file for additional
information.

[urdf-web]: http://wiki.ros.org/urdf/XML
[kindsl-web]: https://robcogenteam.bitbucket.io/rmodel.html
