[TOC]

## :bulb: Overview 

GitHub labels are used for Issues and Pull Requests (PR) and hence it should make clear sense for general usage. The labels shall provide only the important contexts such as State, Type and Priority.

There are many methods of label strategies. For easy understanding and better follow up, a Sane labeling method is recommend. Sane Labeling Scheme classifies the labels into a number of groups and the labels are defined using the group name as prefix. 

## Recommended Labels 

The following are the recommend group of labels and already 
implemented in this [wiki](https://git.smartfactory.de/wiki/bestpractices/-/labels).

### State or Status labels 

The labels are created as per the Sane Labeling Scheme and having a format `status:<name of label>` The following are the examples of recommend status labels with possible color combinations, but the `name of label` as well the label colors can be easily adjusted based on the nature of the project. 

Status labels aim to represent the state of the project and normally only one status label will be applied to one state of the project. For example, it doesn't make sense to have states `in progress` and `abandoned` at the same time for a project. But exceptionally in sometimes states like `completed` and `accepted` can of course  co-exist. 

It is also recommended to write the labels in small letters.

### Type labels

Type labels represents what type of issue the project is facing at present. The type labels can be given in the form `type:<name of label>` . The following are the few examples of possible type labels with possible colors.

### Priority labels

Priority labels are used to categorize the events in the order of emergency. The labels are in the format `priority:<name of label>` . 

Introducing priority labels to a project can sometimes make every event/issue suddenly becomes "absolutely-mission-critical-fix-now-or-the-business-will-fail". Hence it is purely the choice of developers to determine the level of urgency based on these labels.

### :wrench: Managing Labels 

The labels can be managed under the section [ `Project Information -> Labels` ](https://git.smartfactory.de/wiki/bestpractices/-/labels)

### :books: References 

* [Sane labels](https://github.com/seantrane/github-label-presets)
