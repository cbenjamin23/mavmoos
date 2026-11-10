"""Local Docker connection and narrowly scoped release resource cleanup."""
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import tempfile

OWNER_LABEL = 'mavmoos.release.owner'
CONTAINER_ID = re.compile(r'[a-f0-9]{64}')


def need(condition, message):
    if not condition:
        raise ValueError(message)


class LocalDocker:
    def __init__(self, executable='docker'):
        command = shutil.which(executable)
        need(command, 'Docker CLI executable is unavailable')
        self.executable = str(Path(command).resolve(strict=True))
        # Context inspection only reads local configuration. Pin its effective
        # endpoint before doing anything that could contact the Docker daemon.
        observed = subprocess.check_output(
            [self.executable, 'context', 'inspect', '--format', '{{json .Endpoints.docker.Host}}'],
            text=True, stderr=subprocess.PIPE, timeout=10)
        host = json.loads(observed)
        need(isinstance(host, str) and host.startswith('unix:///') and
             not re.search(r'[\x00-\x1f\x7f?#]', host), 'release Docker requires an absolute local Unix socket')
        self.path = Path(host.removeprefix('unix://')).resolve(strict=True)
        self.socket_identity = self.check_socket()
        self.host = 'unix://' + str(self.path)
        self.env = {key: value for key, value in os.environ.items() if not key.startswith('DOCKER_')}
        self.config = tempfile.TemporaryDirectory(prefix='mavmoos-release-docker-')

    def check_socket(self):
        observed = self.path.lstat()
        need(stat.S_ISSOCK(observed.st_mode), 'release Docker endpoint is not a Unix socket')
        return observed.st_dev, observed.st_ino

    def argv(self, *args):
        need(self.check_socket() == self.socket_identity, 'release Docker socket changed during the run')
        return [self.executable, '--config', self.config.name, '--host', self.host, *map(str, args)]

    def capture(self, *args, timeout=20):
        return subprocess.check_output(self.argv(*args), env=self.env, text=True, timeout=timeout).strip()

    def evidence(self):
        device, inode = self.socket_identity
        return {'executable': self.executable, 'host': self.host, 'device': device, 'inode': inode,
                'isolated_cli_config': True, 'docker_environment_scrubbed': True}

    def close(self):
        self.config.cleanup()


def owned_container(row, name, owner, expected_id):
    identity = row['Id']
    labels = row['Config'].get('Labels') or {}
    need(CONTAINER_ID.fullmatch(identity) and (expected_id is None or identity == expected_id) and
         row['Name'] == '/' + name and labels.get(OWNER_LABEL) == owner,
         'release container identity differs')
    return identity


def remove_owned_container(docker, name, owner, expected_id):
    need(re.fullmatch(r'[a-z0-9][a-z0-9-]*', name), 'invalid release container name')
    exact_name = f'name=^/{name}$'
    listing = ('container', 'ls', '-aq', '--no-trunc', '--filter')
    selector = (*listing, exact_name, '--filter', f'label={OWNER_LABEL}={owner}')
    candidates = docker(*selector).splitlines()
    if not candidates:
        need(expected_id is None or not docker(*listing, 'id=' + expected_id),
             'release container lost its name or owner label')
        return None
    need(len(candidates) == 1 and CONTAINER_ID.fullmatch(candidates[0]), 'ambiguous release container identity')
    rows = json.loads(docker('container', 'inspect', candidates[0]))
    need(len(rows) == 1, 'Docker did not return one owned container')
    identity = owned_container(rows[0], name, owner, expected_id or candidates[0])
    need(identity == candidates[0], 'listed and inspected container IDs differ')
    docker('container', 'rm', '--force', identity)
    need(not docker(*selector) and not docker(*listing, 'id=' + identity), 'owned release container remains')
    return identity


def owned_volume(row, name, owner):
    labels = row.get('Labels') or {}
    need(row['Name'] == name and labels.get(OWNER_LABEL) == owner,
         'release work volume identity differs')


def remove_owned_volume(docker, name, owner):
    listing = ('volume', 'ls', '--format', '{{.Name}}', '--filter', f'label={OWNER_LABEL}={owner}')
    if name not in docker(*listing).splitlines():
        return False
    rows = json.loads(docker('volume', 'inspect', name))
    need(len(rows) == 1, 'Docker did not return one owned work volume')
    owned_volume(rows[0], name, owner)
    docker('volume', 'rm', name)
    need(name not in docker(*listing).splitlines(), 'owned release work volume remains')
    return True
