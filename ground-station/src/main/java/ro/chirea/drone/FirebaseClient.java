package ro.chirea.drone;

import java.io.IOException;
import java.net.URI;
import java.net.URLEncoder;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.Map;

/**
 * Firebase Realtime Database REST client with e-mail/password authentication.
 *
 * Replaces the 2022 chain  Java -> log.txt -> .bat -> Python -> Firebase  (one new
 * process per key press, absolute Windows paths, open database) by direct HTTPS
 * calls over one persistent connection.
 *
 * SPDX-License-Identifier: MIT
 */
public class FirebaseClient {
    private static final String AUTH =
            "https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword?key=";

    private final HttpClient http = HttpClient.newBuilder()
            .connectTimeout(Duration.ofSeconds(4)).build();
    private final String dbUrl;
    private final String apiKey;
    private String idToken;
    private long expiresAt;
    private String email, password;

    public FirebaseClient(String dbUrl, String apiKey) {
        this.dbUrl = dbUrl.replaceAll("/+$", "");
        this.apiKey = apiKey;
    }

    public synchronized void signIn(String email, String password) throws IOException, InterruptedException {
        this.email = email;
        this.password = password;
        String body = Json.write(Map.of("email", email, "password", password, "returnSecureToken", true));
        HttpResponse<String> r = http.send(HttpRequest.newBuilder(URI.create(AUTH + apiKey))
                .timeout(Duration.ofSeconds(6))
                .header("Content-Type", "application/json")
                .POST(HttpRequest.BodyPublishers.ofString(body)).build(), HttpResponse.BodyHandlers.ofString());
        if (r.statusCode() != 200) {
            throw new IOException("authentication failed (" + r.statusCode() + "): " + errorMessage(r.body()));
        }
        Map<?, ?> m = (Map<?, ?>) Json.parse(r.body());
        idToken = (String) m.get("idToken");
        expiresAt = System.currentTimeMillis() + (Long.parseLong((String) m.get("expiresIn")) - 60) * 1000L;
    }

    public boolean isSignedIn() { return idToken != null; }

    private synchronized String token() throws IOException, InterruptedException {
        if (idToken == null) throw new IOException("not signed in");
        if (System.currentTimeMillis() > expiresAt) signIn(email, password);  // tokens live 1 h
        return URLEncoder.encode(idToken, StandardCharsets.UTF_8);
    }

    private URI uri(String path) throws IOException, InterruptedException {
        return URI.create(dbUrl + "/" + path.replaceAll("^/+", "") + ".json?auth=" + token());
    }

    public void put(String path, Object data) throws IOException, InterruptedException {
        send(HttpRequest.newBuilder(uri(path)).timeout(Duration.ofSeconds(3))
                .PUT(HttpRequest.BodyPublishers.ofString(Json.write(data))).build());
    }

    public Object get(String path) throws IOException, InterruptedException {
        return Json.parse(send(HttpRequest.newBuilder(uri(path)).timeout(Duration.ofSeconds(3)).GET().build()));
    }

    private String send(HttpRequest req) throws IOException, InterruptedException {
        HttpResponse<String> r = http.send(req, HttpResponse.BodyHandlers.ofString());
        if (r.statusCode() / 100 != 2) throw new IOException("HTTP " + r.statusCode() + ": " + errorMessage(r.body()));
        return r.body();
    }

    private static String errorMessage(String body) {
        try {
            Object o = Json.parse(body);
            if (o instanceof Map) {
                Object e = ((Map<?, ?>) o).get("error");
                if (e instanceof Map) return String.valueOf(((Map<?, ?>) e).get("message"));
                return String.valueOf(e);
            }
        } catch (RuntimeException ignored) { }
        return body;
    }
}
