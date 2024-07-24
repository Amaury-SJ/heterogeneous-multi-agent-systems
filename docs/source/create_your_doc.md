# Create your open-source scientific documentation

This tutorial makes it easy to create scientific documentation for your project and publish it online, free of charge and open-source.
The documentation for this site was created using Markdown, Sphinx and Read the Docs.

## Tools used: Markdown, Sphinx and Read the Docs

To write the content of our documentation, we use [Sphinx], a free documentation generator. It supports reStructuredText or Markdown.
reStructuredText is the default plaintext markup language used by Sphinx. [Markdown] is a lightweight markup language with a simplistic plain text formatting syntax. To use Markdown in Sphinx, we use the [MyST] community tool.

[Read the Docs] allows you to generate and host your documentation automatically. Completely open source, there is an enterprise and a community version. The latter can host our site free of charge from public repositories such as GitHub or GitLab.

## Tutorial

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
