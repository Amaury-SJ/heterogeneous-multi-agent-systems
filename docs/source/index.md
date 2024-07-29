% Heterogeneous multi-agent systems documentation master file, created by
% sphinx-quickstart on Fri Jul  5 17:27:05 2024.
% You can adapt this file completely to your liking, but it should at least
% contain the root `toctree` directive.

# Welcome to Heterogeneous multi-agent systems's documentation!

```{warning}
This documentation is under heavy development and redaction.

Actually, much of this research is unpublished, so the content is hidden until the thesis is published and new research papers accepted.
```

## Creating recipes

To retrieve a list of random ingredients,
you can use the ``lumache.get_random_ingredients()`` function:

```{eval-rst}
.. autofunction:: lumache.get_random_ingredients
```

This is a cross reference {py:func}`lumache.get_random_ingredients`

The exception:

```{eval-rst}
.. autoexception:: lumache.InvalidKindError
```
{kbd}`Ctrl` + {kbd}`Space`

```{eval-rst}
.. autosummary::
    :nosignatures:

    myst_parser.sphinx_ext.main.setup_sphinx
    myst_parser.sphinx_ext.main.create_myst_config
```

## Contents

```{toctree}
:maxdepth: 1

gps-rtk
zotero
create_your_doc
```