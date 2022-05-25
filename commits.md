[TOC]

## Commits and Publications

 - Commits: 
   - store modifications, additions and deletions in your local repository. 
   - Not public to other users
 - Publication: 
   - Push your changes to remote repository. Publications should be done less frequently than commits. 
   - Before publishing aa large amount of commits, collaps them -> readability

## When to commit?

 - commit often to your local repository to keep a backup of your work in your repository
 - keep your commits readable

## When to push?

 - push when you're done with a certain chunk of work that is closed in itself

## Linking commits to Issues

Issues are generated with an issue ID. If this ID is used in a commit, the commit is automatically linked to the issue. In the issue, there will be a message which states, that @user mentioned the issue in a commit. This helps to categorize commits in terms of the issues they're covering.
Mentioning an issue is done by simply adding the ID to your commit message. 
The ID is given as a # followed by a number: #1, #2, #3, ...

## Commit Syntax

A good way to structure your commit messages is, by using expressions to classify the nature of your commits. This can be done in written form:
 - 'Added a new feature...'
 - 'Modified ai trianing...'
 - 'Deleted deprecated classes...'
 - 'Fixed an issue, where...'

Another way to write those classifications is by using tags, such as: 
 - [ADD]
 - [MOD]
 - [DEL]
 - [FIX]

## Commit Syntax - including linkage

You can combine the issue-linking and the syntax by using a commit form that looks something like this:
 - [ADD] #1: Added a new feature ...
 - [MOD] #1: Modified a feature ...
 - [DEL] #1: Deleted a file, ...
 - [FIX] #1: Fixed a bug, where ...
