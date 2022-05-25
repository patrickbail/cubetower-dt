## All you want to know about Markdown Syntax

## :tada: Making the Most out of Markdown Language
Here is a brief overview in using the Markdown language in making your documentation easier and efficient. 

## Table of Contents
- [Text Formatting](#textformatting)
- [Creating Table](#creatingTable)
- [Fenced Code Blocks](#fencedCode)
- [Footnotes](#footnotes)
- [Heading IDs](#headinglD)
- [Linking to Headings](#headinglinking)
- [Strike Through Texts](#strikethorugh)
- [List of Tasks](#taskLists)
- [Emojis](#emojis)
- [URL Linking](#urlLinking)
- [References](#reference)

#
### Text Formatting <a name="textformatting"></a>

The following text formatting syntax can be used.

![image](uploads/110338e87417a16911018d975d54a4bc/image.png)

#### Writing Bold
 Text can be converted to bold using two stars (**)

Best practice: 

 ![image](uploads/9b4af67d1da930c3ffba0543574be105/image.png)

#### Writing Italics
Text can be converted to italics using underscores (_) or using single star (*)

Best Practice:

![image](uploads/bf604d0e2ae7a6ffc00967c0668f7611/image.png)

#
### Creating Table <a name="creatingTable"></a>
To add a table, use three or more hyphens (---) to create each column’s header, and use pipes (|) to separate each column. 

    | Syntax | Description |
    | --- | ----------- |
    | Header | Title |
    | Paragraph | Text |

The above syntax will give you the following table
| Syntax | Description |
| --- | ----------- |
| Header | Title |
| Paragraph | Text |

To know more about Table alignment & Text formatting in Table visit [here](https://www.markdownguide.org/extended-syntax/#alignment)

#
### Fenced Code Blocks <a name="fencedCode"></a>
Code blocks can be created in the basic Markdown syntax by indenting lines by four spaces or one tab. 

More conveniently a fenced code block, depending on the Markdown  editor, can be created by using three backticks (```) or three tildes (~~~) on the lines before and after the text you want to have in the code block. This method does not require indexing for individual lines.

``` 
This is a code block
```

Similarly a text highlight like this: `this is a highlighted text` can be created simply giving a backtick (`) before and after the text.

#
### Footnotes <a name="footnotes"></a>
To create a footnote reference, add a caret and an identifier inside square brackets ([^1]). Identifiers can be numbers or words, but they can’t contain spaces or tabs. The identifiers are used only for reference and the final footnotes are numbered sequentially. Below is an example footnote. 
```
Here's a simple footnote,[^1] and here's a longer one.[^bignote]

[^1]: This is the first footnote.

[^bignote]: Here's one with multiple paragraphs and code.

    Indent paragraphs to include them in the footnote.

    `{ my code }`

    Add as many paragraphs as you like.
```
The rendered output of the above syntax will look like this:

![image](uploads/808e417de2276dd95d7f8b76aa8c20ad/image.png)


#
### Heading IDs <a name="headinglD"></a>
Adding custom IDs allows to link directly to the headings in a page and the same can be modified with CSS. To add a custom heading ID, enclose the custom ID in curly brackets on the same line as the heading.

```
### My Heading {#custom-id}
```
The HTML looks like this:
```
<h3 id="custom-id">My Great Heading</h3>
```
#
### Linking to Headings <a name="headinglinking"></a>
The headings with a custom ID can be linked by creating a standard link with a number sign (#) followed by a unique ID. The following table shows the usage. Here the `My Heading` is connected using the link refered by the `custom-id`
|Markdown|HTML|
|--------|-----------|
|`[My heading](#custom-id)`|`My Heading <a name="custom-id"></a>`|


#
### Strike Through Texts <a name="strikethorugh"></a>
To strikethrough words, use two tide symbols (~~) before and after the words

```
~~The Earth is falt.~~ The earth is round.
```
This gives: 

~~The Earth is falt.~~ The earth is round.

#
### List of Tasks <a name="taskLists"></a>
Task lists gives a list of tasks with checkboxes. To create a task list, add dashes (-) and square brackets with a space ([ ]) in front of the list items. To select a checkbox, add an x between the brackets ([x])
```
- [x] Task 1
- [ ] Task 2
- [ ] Task 3
```
This gives:
- [x] Task 1
- [ ] Task 2
- [ ] Task 3

#
### Emojis <a name="emojis"></a>
Emojis are mostly useful to interact with the reader more conveniently. Emojis can be applied in Markdown languge in two methods: simply by copy and paste the emojis or by typing emoji shortcuts.

Here are few of the resources for Emojis to copy:
- [Emjojipedia](https://emojipedia.org/)
- [Complete List of Markdown Emojis : Git Repo](https://gist.github.com/rxaviers/7360908)

Or you can use the shortcuts like this:
```
Gone camping! :tent: Be back soon.
That is so funny! :joy:
```
This gives:

Gone camping! :tent: Be back soon.
That is so funny! :joy:

#
### URL Linking <a name="urlLinking"></a>

The external websites can be linked to headings using the same method. The below example links the Textto the given external address.
```
[Text](www.<address_to_external_website>)
```

#
## References <a name="reference"></a>

1. [Extended Guide on Markdown Languge syntax](https://www.markdownguide.org/extended-syntax/)
2. [Emojis Source: Emojipedia](https://emojipedia.org/)
3. [Complete List of Markdown Emojis : Git Repo](https://gist.github.com/rxaviers/7360908)
4. [Basic Markdown](https://www.markdownguide.org/basic-syntax/)