# Setup

Steps to get this repo running on a fresh machine.

## 1. Clone

Fork the repo to your own GitHub account, then clone over HTTPS:

```bash
git clone https://github.com/<your-username>/wato_asd_navigation.git
cd wato_asd_navigation
```

Forking makes `origin` point at your fork, so `git push` works with no extra setup.

## 2. Install Docker

Follow Docker's official guide, [Install using the apt repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) (also linked from the [docs](https://wiki.watonomous.ca/admission_assignments/asd_admission_assignment/#setup)).

Set up Docker's apt repository:

```bash
# Add Docker's official GPG key
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to apt sources
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
```

Then install the Docker packages:

```bash
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Start Docker:

```bash
sudo systemctl enable --now docker
```

Add yourself to the `docker` group so you can run it without `sudo`:

```bash
sudo usermod -aG docker $USER
newgrp docker      # apply the group in this shell (or log out and back in)
```

Verify and clean up:

```bash
docker run --rm hello-world      # --rm removes the test container
docker rmi hello-world           # remove the test image too
```
