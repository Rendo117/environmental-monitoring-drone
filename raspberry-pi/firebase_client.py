"""
firebase_client.py — minimal Firebase Realtime Database REST client with
e-mail/password authentication (no third-party Firebase SDK required).

Replaces the unmaintained `python-firebase` package used in 2022 and the
publicly writable database URL that was hard-coded in the scripts.

SPDX-License-Identifier: MIT
"""
from __future__ import annotations

import time

import requests

AUTH_URL = "https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword?key={key}"
REFRESH_URL = "https://securetoken.googleapis.com/v1/token?key={key}"


class FirebaseRTDB:
    def __init__(self, db_url: str, api_key: str, email: str, password: str, timeout: float = 3.0):
        self.db_url = db_url.rstrip("/")
        self.api_key = api_key
        self.email = email
        self.password = password
        self.timeout = timeout
        self.session = requests.Session()     # keep-alive: one TLS handshake, lower latency
        self.id_token: str | None = None
        self.refresh_token: str | None = None
        self.expires_at = 0.0

    # -- authentication -----------------------------------------------------
    def sign_in(self) -> None:
        r = self.session.post(AUTH_URL.format(key=self.api_key), timeout=self.timeout, json={
            "email": self.email, "password": self.password, "returnSecureToken": True})
        r.raise_for_status()
        d = r.json()
        self._store(d["idToken"], d["refreshToken"], int(d["expiresIn"]))

    def _refresh(self) -> None:
        r = self.session.post(REFRESH_URL.format(key=self.api_key), timeout=self.timeout, data={
            "grant_type": "refresh_token", "refresh_token": self.refresh_token})
        r.raise_for_status()
        d = r.json()
        self._store(d["id_token"], d["refresh_token"], int(d["expires_in"]))

    def _store(self, id_token: str, refresh_token: str, expires_in: int) -> None:
        self.id_token, self.refresh_token = id_token, refresh_token
        self.expires_at = time.time() + expires_in - 60     # renew 1 min early

    def _auth(self) -> dict:
        if self.id_token is None:
            self.sign_in()
        elif time.time() > self.expires_at:
            self._refresh()
        return {"auth": self.id_token}

    # -- data ---------------------------------------------------------------
    def _url(self, path: str) -> str:
        return f"{self.db_url}/{path.strip('/')}.json"

    def get(self, path: str):
        r = self.session.get(self._url(path), params=self._auth(), timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def put(self, path: str, data) -> None:
        r = self.session.put(self._url(path), params=self._auth(), json=data, timeout=self.timeout)
        r.raise_for_status()

    def post(self, path: str, data) -> str:
        r = self.session.post(self._url(path), params=self._auth(), json=data, timeout=self.timeout)
        r.raise_for_status()
        return r.json()["name"]

    def patch(self, path: str, data) -> None:
        r = self.session.patch(self._url(path), params=self._auth(), json=data, timeout=self.timeout)
        r.raise_for_status()
