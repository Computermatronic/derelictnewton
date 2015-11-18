DerelictNewton
=============

Dynamic Bindings to the [Newton Dynamics physics library][1] version 3.03 for the D Programming Language.

Please see the pages [Building and Linking Derelict][2] and [Using Derelict][3], in the Derelict documentation, for information on how to build DerelictNewton and load the Newton Dynamics libraries at run time. In the meantime, here's some sample code

```D
// This example shows how to load the DerelictNewton bindings.
import derelict.newton.newton;

void main() {
    // This example shows how to load the Newton library. 
    DerelictNewton.load();

    // Now Newton Dynamics functions for all of the Newton Dynamics libraries can be called.
    ...
}
```

[1]: http://newtondynamics.com
[2]: http://derelictorg.github.io/compiling.html
[3]: http://derelictorg.github.io/using.html
