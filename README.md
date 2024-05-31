# Learning Machines Robobo

This is the GitHub repository for the Learning Machines course.

If you're a student, everything you need for the course itself is in the [examples](https://github.com/ci-group/learning_machines_robobo/tree/master/examples) directory. It contains all the documentation and code you need. Just clone this repository, cd into examples, and start at that readme, which will guide you through the whole thing.

### There is an issue/bug with the code.

If you find a problem with the code, please create an issue [here](https://github.com/ci-group/learning_machines_robobo/issues) on the GitHub repository. For this, please make sure you include these three things:

- Some example code with instructions on how to run and include it. This should preferably be a minimum failing example, just linking to your entire assignment won't cut it. You might think this is a lot of extra work, but the amount of times I personally found a bug in my own code by trying to construct an example like this is staggering. It's a really good test to make sure that what you think is happening is actually what is happening. Also, it helps whoever wants to fix it understand what is going wrong.
- The behaviour you would expect from this minimum failing example. No "it should work," be specific in what output you expect.
- The actual behaviour you observed, and that anyone can observe by running the example code you provided. Here, also provide the platform you ran it under, in case it cannot be reproduced.

## Contributing / maintaining

If you are working on the project, you should notice that all code you write yourself should be in `maintained/`. All code in the examples is automatically generated (or, well, copied over) from there. This architecture is quite weird, but I couldn't find a better option. Because everything is ROS, it is hard to distribute the individual packages without having to teach students how to install ROS packages, making the code too much of a black box. Alternatively, having only one template (e.g. only `full_project_setup`), which is what was here before, has issues with documentation, as that makes it quite a large project to just dump students into. With the way it currently is, everything is in one place while maintaining, but students can still cd from example to example to explore the codebase.

To work on the project, first, `cd` into maintained. Here, you can edit the code and test it, (though it might be easier to test it inside the directory of the relevant example.) Once you are confident it is good, you can type `python3 ./build.py`, this will copy all files over to the right examples. If you created new scripts or catkin packages, this build script is also where to make the needed changes to always copy over the files to the appropriate examples. The build script will ask for confirmation for every file it wants to delete. If you're confident everything is backed up, you can pass the `-y` flag to answer yes to all prompts.

After you have built, it is important to go through the READMEs in the examples, and assert all of them are still correct.

If you change anything with the docker setup or run scripts it is important to test everything under Linux (X11), Windows, and MacOS before pushing to master, making sure the behaviour is consistent.

If you changed the Lua scripts, make sure to update all affected models and scenes to match. This is quite tedious, and if you find a way to automate it, please let me know.
