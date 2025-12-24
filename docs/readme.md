# Docs Explanation

This directory defines the tools needed to **generate** the documentation
for `brom_drake`. It does not contain the raw documentation, but you can
generate it yourself with the following make target:

```shell
make generate-and-build-docs
```

which you can run in your terminal.

For more details, see below:

## Output Format of the Docs

The docs are generated as a set of HTML files. They will be created in a new `build/html` directory in this directory after running the appropriate make target.

That directory is stand-alone and doesn't need a server running to display its results. Open up any file in there and you should be able to see the file as it will appear online (on GitHub pages).

## Scripts in this directory

There are two scripts in this directory:

1. `generate_docs.py`: Which generates documentation based on the structure of the `brom_drake` package.
2. `update_version_in_conf.py`: Which will update the version (i.e., the `release` field) in the `conf.py` file which is used in a lot of Sphinx's generation process.

Check the make target described above for example usage of both of these scripts.