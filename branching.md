[TOC]


# Branching
Branching is an important aspect of every git projects. It allows to control different versions of your code. As one of a main purpose for our git workflow is to have certainty about stable code products, while being able to maintain an agile software development process, we are using a [Gitflow Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow) branching model. The idea of this branching approach is, to ensure that the main branch is always holding a stable version of the complete software system that is under development. Features are built from development branch, and later released into the main branch. 

## Basic Workflow Concept 

![image](/images/branching.png)

*Main*

*Main* is containing stable states of code and project environments and it's commits represent release versions, as depicted in the image above. This enables fallback mechanism to earlier releases without cluttering the branch structure.

*Development* 

The *Development* branch is used as entry point for new features that have to be developed. Finished work, is pushed and merged back into *Development*, from which a release is developed. 

*Release* 

For each release, a *Release* branch is used to avoid side effects from changes that occur on *Development* during preparation of the release. After everything is finished an tested, *Release* is merged into *Main*. 

*Feature*
A *Feature* branch is created for each feature that has to be created for a system. The repective branch is created from the current state in the dev branch.


## Advanced Concept: Nested Projects (WIP)

Sometimes it is convinient to reference a project instead of copying it's content. This can become handy when your project evolves as well as the project you referred to in your repository. Instead of copying the progress each and every time one can reference the project - or a certain commit. 

![image](/images/nested_branching.png)














*Step 1: Make sure your project is up-to-date*
----------------

Before creating a new branch, ensure that the project is on the latest version by pulling it:

    :::git CLI
    git pull


*Step 2: go to development branch*
----------------

A branch can be selected on the git repository website or via command in git:

    :::git CLI
    git checkout <branch>

To work on the development branch, the command would be

    :::git CLI
    git checkout development


*Step 3: create a new branch*
----------------


Creating a new branch can be done via the git webpage or as a command in git:

    :::git CLI
    git checkout -b <new-branch-name>



*Step 4: Update remote repository*
----------------

When pushing the first time on your new branch, you'll have to create a connection from your local branch to a remote branch. Your very first push request should look like this:

    :::git CLI
    git push--set-upstream origin <new-branch-name>




After your branch is established on the remote end you can simply use


    :::git CLI
    git push