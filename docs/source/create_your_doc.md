# Create your open-source scientific documentation

This tutorial makes it easy to create scientific documentation for your project and publish it online, free of charge and open-source.
The documentation for this site was created using Markdown, Sphinx and Read the Docs.

## Markdown, Sphinx and Read the Docs

### Tools used

To write the content of our documentation, we use [Sphinx], a free documentation generator. It supports reStructuredText or Markdown.
reStructuredText is the default plaintext markup language used by Sphinx. [Markdown] is a lightweight markup language with a simplistic plain text formatting syntax. To use Markdown in Sphinx, we use the [MyST] community tool.

[Read the Docs] allows you to generate and host your documentation automatically. Completely open source, there is an enterprise and a community version. The latter can host our static site free of charge from public repositories such as GitHub or GitLab.

```{tip}
For writing documentation, we use the official documentation of [MyST-parser](https://myst-parser.readthedocs.io/en/latest/syntax/typography.html) and [MyST](https://mystmd.org/guide/typography).
```

### Other tools for your documentation

There are other easy ways to create scientific documentation:
- on the GitHub and GitLab platforms, you can create a wiki section directly in your directory
- solutions are available for members of French universities, such as the creation of a personal wiki space, or CompACT for creating an electronic portfolio

### Tutorial

From the official Read the Docs [tutorial], we'll first create a folder containing the proposed ReadTheDocs [template].
We'll use Visual Studio Code to simplify various tasks. Let's create a virtual environment with Conda, for example, to create documentation and install the necessary packages. The `codes_and_documentation/.conda` folder doesn't have to be in the same folder as the one containing our codes and documentation `codes_and_documentation/project_name`. You can then select the interpreter in Visual Studio Code via the search bar by typing *>python: Select Interpreter*. As a reminder for the terminal, if you are using a Git bash, conda is already recognized because it is initialized, whereas in PowerShell, conda is not configured by default.

#### Prerequies

You can install the various dependencies noted in the pyproject.toml file, for documentation purposes, with:
```shell
pip install -e .[doc]
```

Or install Sphinx directly:
```shell
pip install sphinx
```

#### Use another theme

We can use another famous theme to create our documentation:
```shell
pip install sphinx_rtd_theme
```

To change the theme, in the `source/conf.py` file, we can set:
```python
html_theme = 'sphinx_rtd_theme'
```

#### Start your documentation

Now, start a sphinx documentation, and create a 'docs' folder, and say **yes** to separate source and directories:
```shell
sphinx-quickstart docs
```

Then we'll build the documentation from the source, so that it can be viewed offline:
```shell
cd docs
make html
```

```{warning}
Two important points:
- These two commands will be used regularly to modify the documentation, and don't forget to be in the `docs` folder beforehand.
- The `make html` command often needs to be run 2 times successively to generate documentation without errors.
```

We select the `docs/build/index.html` file, and we can use a Visual Studio Code extension called `Live Server` to view the page offline, by clicking after installation on the button at the bottom right of the `Go Live` window.

With Visual Studio Code, you can create a Git by initializing a directory, using the `Source Control` tab on the left.

#### Switch reStructuredText to MyST

We want to switch to Markdown, to convert files already written in reStructuredText to MyST:
```shell
pip install rst-to-myst[sphinx]
rst2myst convert docs/**/*.rst
```

We can now delete the old `.rst` files.

After we add the extension in `conf.py`:
```python
extensions = ["myst_parser"]
```

We can use .rst and .md files in the same documentations.


#### Add cross references

To add links to sections and subsections, we can add the extension in `conf.py`:
```python
extensions = ["myst_parser",
              "sphinx.ext.autosectionlabel"]
```

There are two ways of making references:
- either refer to a document, i.e. part of the documentation, using the word `doc` and the file name without the `.dae` extension.
- or refer to a section or sub-section with the word `ref` followed by the section title.
```
{doc}`file_name`
{ref}`Title`
```

#### Tip: add a new page

To add a new section to our documentation, simply create a `codes_and_documentation/project_name/docs/source/new_page.md` file. We also need to add the file name to the `toctree` in our `index.md` :

```
% add three accent before and after to form a block
{toctree}
:maxdepth: 1

first_page
new_page
last_page
```

However, when you add the page and then build with `make html`, the menu on the left may not display the pages correctly. It is therefore preferable to do a `make clean`, which will delete the contents of the build folder, followed by a `make html`.

#### Documenting code automatically

##### 1. First method (recommended): sphinx-autodoc2

This method allows you to document your code automatically. The advantage is that this method analyzes Python code **statically**, so there's no need to install missing libraries such as ROS 2. This method is also recommended by MyST-Parser in its [documentation](https://myst-parser.readthedocs.io/en/latest/syntax/code_and_apis.html#sphinx-autodoc2). Another advantage is that you can automatically generate a list of all the functions and classes of our API.

To install `sphinx-autodoc2`:

```shell
pip install sphinx-autodoc2
```

Next, we'll add to our `docs/source/conf.py` file, specifying our folder containing all our code:

```python
extensions = [
    "myst_parser",
    ...,
    "autodoc2",
]

autodoc2_packages = [
    "../../codes/src/my_first_pckg/my_first_pckg",
    "../../codes/src/my_first_pckg/launch",
    "../../codes/src/my_second_pckg/my_second_pckg",
    ...
]
```

The specified folder name must be a package: it must contain an empty file `__init__.py`. By default, ROS 2 subpackage folders (e.g. `../../codes/src/my_first_pckg/my_first_pckg`) already contain this file. You need to add this empty file to launch folders, for example: `../../codes/src/my_first_pckg/launch`.

We also need to add the direction `apidocs/index` to the toctree of the `docs/source/index.md` file. This will generate an automatic `API Reference` page. The `make html` command will create a `docs/source/apidocs` folder containing an `index.rst` file and the documentation for the indicated packages. It is often necessary to repeat the `make html` command **twice or more**, as warnings appear when the apidocs is not fully generated.

```{tip}
The `docs/source/apidocs` folder can be deleted before a build, as a `make clean` does not remove any additions to apidocs.
```

**Important:**

If you document your Python code using the docstrings format, it may not be displayed correctly. According to the tip in the [sphinx-autodoc2 documentation](https://sphinx-autodoc2.readthedocs.io/en/latest/quickstart.html#using-markdown-myst-docstrings), we need to add to our `docs/source/conf.py` file:

```python
myst_enable_extensions = ["fieldlist"]
```

##### 2. Second method: sphinx.ext.autodoc

```{warning}
This method works and is quick to set up. However, this method analyzes Python code in a **dynamic** way, meaning that all Python libraries (importing code), such as ROS 2 with rclpy, must be installed to generate documentation. This requires the entire project to be installed in order to generate documentation.
```

We're going to use the [simple method](https://myst-parser.readthedocs.io/en/latest/syntax/code_and_apis.html#sphinx-ext-autodoc) with `sphinx.ext.autodoc`, even if it's not the first method [recommended](https://myst-parser.readthedocs.io/en/latest/syntax/code_and_apis.html#sphinx-autodoc2) by the MyST-parser documentation.

We can add the extensions in `conf.py` (don't need to add the extensions to the `requirements.txt` file, because sphinx already installed), with `autodoc` and also `autosummary` to summarize:
```python
extensions = ["myst_parser",
              "sphinx.ext.autosectionlabel",
              "sphinx.ext.autodoc",
              "sphinx.ext.autosummary"]
```

And then in an `eval-rst`, use `.. autofunction:: name_file.function` to obtain directly the docstring documentation of this function for example, and you can link it with `{py:func}'name_file.function'` for example.

Its' possible that your codes are not detected, you must then add the access path in the `conf.py` as [suggested](https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html):

```python
# We added this to document your source code:
import os
import sys
sys.path.insert(0, os.path.abspath('../../codes'))
```

This will also create a folder `codes/__pycache__`.

Here's an example (the punctuation to form the blocks is not respected):

```
To retrieve a list of random ingredients,
you can use the `lumache.get_random_ingredients()` function:

'''{eval-rst}
.. autofunction:: lumache.get_random_ingredients
'''

This is a cross reference {py:func}'lumache.get_random_ingredients'

The exception:

'''{eval-rst}
.. autoexception:: lumache.InvalidKindError
'''

Pour des touches claviers :
{kbd}'Ctrl' + {kbd}'Space'

'''{eval-rst}
.. autosummary::
    :nosignatures:

    myst_parser.sphinx_ext.main.setup_sphinx
    myst_parser.sphinx_ext.main.create_myst_config
    lumache.get_random_ingredients
'''
```

#### Conclusion

At the end of this tutorial, the project tree will look like this:
```yaml
codes_and_documentation
├── .conda
└── project_name
    ├── .gitignore
    ├── .readthedocs.yaml
    ├── pyproject.toml
    ├── README.rst
    ├── codes
    │   ├── my_code1.py
    │   └── ...
    └── docs
        ├── make.bat
        ├── Makefile
        ├── requirements.txt
        ├── source
        │   ├── conf.py
        │   ├── index.md
        │   ├── second_page.md
        │   └── ...
        └── build
            └── ...
```

```{important}
When using new Python requirements, such as `sphinx_rtd_theme` and `myst_parser`, you need to add their names to the `docs/requirements.txt` file, so that when Read The Docs builds its documentation from `.readthedocs.yaml` in the online server, it can install the necessary dependencies.
```

## Code repository

### CITATION.cff file in GitHub

To more easily cite the code repository or associated research paper on GitHub, you can add a `CITATION.cff` file to the root of your repository. A link is automatically added to the repository's home page in the right-hand sidebar of your GitHub. The [GitHub documentation] details the process, and a [site] even allows you to easily generate the file.

### License for the code

Here a link to [licensing a repository].

[MyST]: https://myst-parser.readthedocs.io/en/latest/index.html
[Markdown]: https://www.markdownguide.org/
[Sphinx]: https://sphinx-tutorial.readthedocs.io/
[Read the Docs]: https://docs.readthedocs.io/en/stable/
[tutorial]: https://docs.readthedocs.io/en/stable/tutorial/index.html
[template]: https://github.com/readthedocs/tutorial-template/
[GitHub documentation]: https://docs.github.com/fr/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/about-citation-files
[site]: https://citation-file-format.github.io/
[licensing a repository]: https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/licensing-a-repository