from flask import Flask, request, jsonify

app = Flask(__name__)

# nodes = {
#   "node_name": {
#       "ip": "...",
#       "topics": {
#           "publish": { topic: {port, type} },
#           "subscribe": { topic: {port, type} }
#       }
#   }
# }
nodes = {}

@app.route("/register", methods=["POST"])
def register():
    data = request.json

    name = data.get("name")
    ip = data.get("ip")
    topics = data.get("topics", {})

    if not name or not ip:
        return jsonify({"error": "name and ip required"}), 400

    nodes[name] = {
        "ip": ip,
        "topics": topics
    }

    print(f"[REGISTER] {name} @ {ip}")
    print(f"  publish:   {list(topics.get('publish', {}).keys())}")
    print(f"  subscribe: {list(topics.get('subscribe', {}).keys())}")

    return jsonify({"status": "ok"})

@app.route("/nodes", methods=["GET"])
def get_all_nodes():
    return jsonify(nodes)

@app.route("/node/<name>", methods=["GET"])
def get_node(name):
    return jsonify(nodes.get(name, {}))

@app.route("/topic")
def get_topic():
    topic_name = request.args.get("name")
    if not topic_name:
        return jsonify({"error": "missing topic name"}), 400

    pubs = []
    subs = []

    for name, node in nodes.items():
        topics = node.get("topics", {})

        if topic_name in topics.get("publish", {}):
            pubs.append({
                "node": name,
                "ip": node.get("ip"),
                "direction": "publish",
                **topics["publish"][topic_name]
            })

        if topic_name in topics.get("subscribe", {}):
            subs.append({
                "node": name,
                "ip": node.get("ip"),
                "direction": "subscribe",
                **topics["subscribe"][topic_name]
            })

    return jsonify({
        "topic": topic_name,
        "publishers": pubs,
        "subscribers": subs

    })

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=80)

