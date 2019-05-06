
try:
    import artifactory_utils
except:
    pass
else:
    dependencies = [
        artifactory_utils.ArtifactSelector(
            project="Toolchain-Release",
            revision="develop",
            version="^4.6",
            debug=False,
            stable_required=True),
        artifactory_utils.ArtifactSelector(
            project="linaro-linux-gnu",
            revision="master",
            version="5.3.1.0",
            debug=False,
            stable_required=True),
        artifactory_utils.ArtifactSelector(
            project="sgx_kernel_module_source",
            revision="master",
            version="0.0.0.0",
            debug=False,
            stable_required=True),
    ]
