"""Upload Black Panther 50mm Quad-EDF STEP assembly directly into Onshape via REST API."""

import json
import os
import sys
import time
import requests

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def load_env():
    env = {}
    env_path = os.path.join(BASE_DIR, ".env")
    with open(env_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                env[k.strip()] = v.strip()
    return env


def main():
    env = load_env()
    base = env.get("ONSHAPE_BASE_URL", "https://cad.onshape.com").rstrip("/")
    auth = (env["ONSHAPE_ACCESS_KEY"], env["ONSHAPE_SECRET_KEY"])
    headers = {"Accept": "application/json"}

    r = requests.get(f"{base}/api/v6/users/sessioninfo", auth=auth, headers=headers, timeout=30)
    r.raise_for_status()
    user = r.json()
    print(f"CONNECTED_USER: {user.get('name')} ({user.get('email')})")

    step_name = sys.argv[1] if len(sys.argv) > 1 else "black_panther_quad_edf.step"
    doc_title = sys.argv[2] if len(sys.argv) > 2 else "Black Panther 50mm Quad-EDF VTOL"

    doc_resp = requests.post(
        f"{base}/api/v6/documents",
        auth=auth,
        headers={**headers, "Content-Type": "application/json"},
        json={"name": doc_title, "isPublic": False},
        timeout=30,
    )
    doc_resp.raise_for_status()
    doc = doc_resp.json()
    did = doc["id"]
    wid = doc["defaultWorkspace"]["id"]
    print(f"CREATED_DOC: https://cad.onshape.com/documents/{did}/w/{wid}")

    step_path = step_name if os.path.isabs(step_name) else os.path.join(BASE_DIR, step_name)
    with open(step_path, "rb") as sf:
        up = requests.post(
            f"{base}/api/v6/blobelements/d/{did}/w/{wid}",
            auth=auth,
            headers=headers,
            files={"file": (os.path.basename(step_path), sf, "application/octet-stream")},
            data={
                "translate": "true",
                "flattenAssemblies": "false",
                "yAxisIsUp": "false",
                "storeInDocument": "true",
            },
            timeout=120,
        )
    up.raise_for_status()
    tr = up.json()
    tid = tr.get("translationId")
    print(f"UPLOAD_OK: blob_id={tr.get('id')} translation_id={tid}")

    if tid:
        for _ in range(45):
            st = requests.get(f"{base}/api/v6/translations/{tid}", auth=auth, headers=headers, timeout=30)
            st.raise_for_status()
            info = st.json()
            state = info.get("requestState")
            if state in ("DONE", "FAILED"):
                print(f"TRANSLATION_FINAL: {state} | elements={info.get('resultElementIds')}")
                if state == "FAILED":
                    print("FAIL_REASON:", info.get("failureReason"))
                break
            time.sleep(2)
    else:
        time.sleep(5)

    elems = requests.get(f"{base}/api/v6/documents/d/{did}/w/{wid}/elements", auth=auth, headers=headers, timeout=30)
    elems.raise_for_status()
    for el in elems.json():
        print(f"ELEMENT: {el.get('name')} ({el.get('type')}) -> https://cad.onshape.com/documents/{did}/w/{wid}/e/{el.get('id')}")


if __name__ == "__main__":
    main()
