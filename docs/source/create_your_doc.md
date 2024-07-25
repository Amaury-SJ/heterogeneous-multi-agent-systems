# Create your open-source scientific documentation

This tutorial makes it easy to create scientific documentation for your project and publish it online, free of charge and open-source.
The documentation for this site was created using Markdown, Sphinx and Read the Docs.

## Markdown, Sphinx and Read the Docs

### Tools used

To write the content of our documentation, we use [Sphinx], a free documentation generator. It supports reStructuredText or Markdown.
reStructuredText is the default plaintext markup language used by Sphinx. [Markdown] is a lightweight markup language with a simplistic plain text formatting syntax. To use Markdown in Sphinx, we use the [MyST] community tool.

[Read the Docs] allows you to generate and host your documentation automatically. Completely open source, there is an enterprise and a community version. The latter can host our site free of charge from public repositories such as GitHub or GitLab.

### Tutorial

From the official Read the Docs [tutorial], we'll first create a folder containing the proposed ReadTheDocs [template].
We'll use Visual Studio Code to simplify various tasks. Let's create a virtual environment with Conda, for example, to create documentation and install the necessary packages. The `project/.conda` folder doesn't have to be in the same folder as the one containing our codes and documentation `project/codes_and_docs`. You can then select the interpreter in Visual Studio Code via the search bar by typing *>python: Select Interpreter*

You can install the various dependencies noted in the pyproject.toml file, for documentation purposes, with:
```shell
pip install -e .[doc]
```

Or install Sphinx directly:
```shell
pip install sphinx
```

We can use another famous theme to create our documentation:
```shell
pip install sphinx_rtd_theme
```

To change the theme, in the `source/conf.py` file, we can set:
```python
html_theme = 'sphinx_rtd_theme'
```

Now, start a sphinx documentation, and create a 'docs' folder, and say **yes** to separate source and directories:
```shell
sphinx-quickstart docs
```

Then we'll build the documentation from the source, so that it can be viewed offline:
```shell
cd docs
make html
```

We select the `docs/build/index.html` file, and we can use a Visual Studio Code extension called `Live Server` to view the page offline, by clicking after installation on the button at the bottom right of the `Go Live` window.

With Visual Studio Code, you can create a Git by initializing a directory, using the `Source Control` tab on the left.

We want to switch to Markdown, to convert files already written in reStructuredText to MyST:
```shell
pip install rst-to-myst[sphinx]
rst2myst convert docs/**/*.rst
```

We can now delete the old `.rst` files.

To add links to sections and subsections, we can add the extension in `conf.py`:
```python
extensions = ["myst_parser",
              "sphinx.ext.autosectionlabel"]
```

At the end of this tutorial, the project tree will look like this:
```yaml
project
    .conda
    codes_and_documentation
        .gitignore
        .readthedocs.yaml
        pyproject.toml
        README.rst
        codes
            my_code1.py
            ...
        docs
            make.bat
            Makefile
            requirements.txt
            source
                conf.py
                index.md
                second_page.md
                ...
            build
                ...
```

```{note}
When using new Python requirements, such as `sphinx_rtd_theme` and `myst_parser`, you need to add their names to the `docs/requirements.txt` file, so that when Read The Docs builds its documentation from `.readthedocs.yaml` in the online server, it can install the necessary dependencies.
```

## Other solutions

There are other easy ways to create scientific documentation:
- on the GitHub and GitLab platforms, you can create a wiki section directly in your directory
- solutions are available for members of French universities, such as the creation of a personal wiki space, or CompACT for creating an electronic portfolio

[MyST]: https://mystmd.org/
[Markdown]: https://www.markdownguide.org/
[Sphinx]: https://sphinx-tutorial.readthedocs.io/
[Read the Docs]: https://docs.readthedocs.io/en/stable/
[tutorial]: https://docs.readthedocs.io/en/stable/tutorial/index.html
[template]: https://github.com/readthedocs/tutorial-template/
