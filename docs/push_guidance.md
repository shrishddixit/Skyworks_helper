# Handling CONNECT Tunnel Restrictions When Pushing to GitHub

The development containers used for these exercises do not provide outbound HTTPS CONNECT
tunnels. As a result, `git push` commands to Git hosting providers (GitHub, GitLab, etc.)
fail with errors such as:

```
fatal: unable to access 'https://github.com/<user>/<repo>/': The HTTP proxy server returned an invalid response to the CONNECT request
```

This limitation is environmental—there is no repository-side configuration that can work
around it from within the container. To publish your commits:

1. **Fetch the local changes.**
   * Run `git status` and `git log` to confirm the commit(s) you want to export.
2. **Bundle or generate patches.**
   * `git bundle create skyworks_helper.bundle HEAD` creates a single-file archive.
   * Alternatively, `git format-patch origin/main..HEAD` emits standard patch files.
3. **Transfer the bundle/patches out of the container.**
   * Use the VS Code download command, `docker cp`, or any other file transfer method
dependent on your environment.
4. **Apply and push from a network-enabled machine.**
   * On your workstation, clone `https://github.com/shrishddixit/Skyworks_helper`.
   * Apply the bundle: `git pull path/to/skyworks_helper.bundle` (or `git am` for patches).
   * Push normally: `git push origin <branch>`.

### Why not use SSH or PAT tokens inside the container?
Even with a personal access token or SSH key, the proxy still blocks outbound CONNECT
tunnels, so authentication does not help. The only reliable workaround is to export the
changes and push from a machine that has unrestricted outbound HTTPS access.

### Verifying exported history
After applying the bundle or patches on your local machine, re-run `git status` and `git
log` to ensure the history matches before pushing. This guarantees the GitHub repository
will contain exactly the commits created inside the container.
