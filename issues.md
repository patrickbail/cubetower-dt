## Table of contents
- [Creating an issue](#creating_issue)
- [Issues- Life Cycle](#issue_lifecycle) 
- [Labels](#labels)
- [Assignments](#assignments)

# Issues
The issue section in git is a powerful tool to track development progress, set accountabilities and get a good overview what's going on in your project. The usage of those issues should be integrated within the workflow of your weekly sprint. 


*Creating an Issue*<a name = "creating_issue"></a> 
----------------

Try to create issues for every task you're assigned to. Every issue will be assigned with an ID, which can be seen below the issue name in the overview (red boxes) or in the path when you're looking at an issue (blue box) 

![gitissues](uploads/fcf56db82015320645997d11d9198076/gitissues.png)

If a task is complex or consists out of subtasks, create an issue for every subtask and an issue for the feature that combines them

![gitissues-nested](uploads/5e1900c9878b024fed422a1e5b83c9fc/gitissues-nested.png)

Try to keep your issues short but precise and clean. 

A collection of Markdown instructions to structure your issues can be found here: https://sourceforge.net/p/checkbox/wiki/markdown_syntax/



*Issue - Lifecycle*<a name = "issue_lifecycle"></a> 
----------------

Issues are going through a lifecycle - similar to tasks in a sprint. 

![image](uploads/d4af719bfb693fe92c029439260af604/image.png)

When an issue is created, it is queued with other issues with respect to its priority. 

Once a person starts working on it, it is considered to be in progress. 

Before deploying a finished task it has to be tested and modified if testing showed flaws in the product. 

If the product was tested it has to be integrated to make sure that there are no side-effects to other system parts. 

Before closing an issue, it has to be documented. Note that the documentation should consist out of the most relevant information. A minimum requirement for this task to define the author of the product.

During the entire process it is possible that an issue is rejected, because it falls out of scope of the sprint/product goal, is too hard/complex to achieve or because it becomes deprecated in any other way. If an issue is rejected it can be closed. In some cases it might be a good idea to document the reason of rejection to avoid similar mistakes in the future.

When an issue is done, it can be closed. Closed issues can still be reviewed for future decisions and can therefore be used as basis for upcomming iterations.





*Labels*<a name = "labels"></a> 
----------------
When an issue is created it should keep a specific commonly agreed label, until someone starts working on it. At the moment there are several labels for you to attach to your issues. The section [labels](Git Guidelines/Labels) gives you an overview on the best practices for creating and maintaining labels.


*Assignments*<a name = "assignments"></a> 
----------------
An issue should always have a person assigned to it. The assignee is responsible for the progress of the issue in its *current* state. For testing it is recommended to assign a different person to review your work. As soon as your issue reaches the testing phase, another person should be assigned. Assigned persons will receive an email (by default) as a notification.