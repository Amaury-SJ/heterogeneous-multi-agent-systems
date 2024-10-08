# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Heterogeneous Multi-Agent Systems'
copyright = '2024, Amaury Saint-Jore'
author = 'Amaury Saint-Jore'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# You must add extensions too in requirements.txt file for ReadTheDocs
extensions = ["myst_parser",
              "sphinx.ext.autosectionlabel",
              "sphinx.ext.autodoc",
              "sphinx.ext.autosummary"]

templates_path = ['_templates']
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# We added this to document your source code:
import os
import sys
sys.path.insert(0, os.path.abspath('../../codes'))