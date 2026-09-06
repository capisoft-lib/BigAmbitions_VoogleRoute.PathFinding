"""Render reproducible graph/real C# polyline comparisons (requires matplotlib).

Arguments: BEFORE.csv AFTER.csv BEFORE-routes.json AFTER-routes.json OUTPUT_DIR
Images show waypoint geometry, not screenshots or collision meshes.
"""
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

import repair_deadend_turns as repair

GRAY, BLUE, ORANGE, GREEN, RED = "#8a96a3", "#7b8cca", "#d87814", "#008b6c", "#c33945"


def segment(row):
    a, b = repair.xy(repair.point(row, "from")), repair.xy(repair.point(row, "to"))
    if row["controlX"] and row["controlZ"]:
        c = float(row["controlX"]), float(row["controlZ"])
        return [tuple((1-t)**2*u+2*(1-t)*t*v+t*t*w for u,v,w in zip(a,c,b))
                for t in [i/24 for i in range(25)]]
    return [a, b]


def backdrop(ax, rows, center, radius):
    x,z = center
    groups = defaultdict(list)
    for row in rows:
        points = segment(row)
        if (min(p[0] for p in points)>x+radius or max(p[0] for p in points)<x-radius
                or min(p[1] for p in points)>z+radius or max(p[1] for p in points)<z-radius):
            continue
        elevated = max(float(row["fromY"]), float(row["toY"])) > 3
        color = BLUE if elevated else GRAY
        if row["edgeType"] != "base": color = "#e9bb85"
        groups[color].append(points)
    for color, lines in groups.items():
        ax.add_collection(LineCollection(lines, colors=color, linewidths=1, alpha=.8))
    ax.set(xlim=(x-radius,x+radius), ylim=(z-radius,z+radius), xlabel="X monde (m)", ylabel="Z monde (m)")
    ax.set_aspect("equal")
    ax.grid(alpha=.15)
    ax.ticklabel_format(useOffset=False, style="plain")


def route(ax, result, color):
    if not result["found"]: return
    points = result["points"]
    ax.plot([p[0] for p in points], [p[2] for p in points], color=color, lw=2.8, zorder=6)
    ax.scatter(points[0][0], points[0][2], marker="o", color=color, s=24, zorder=7)
    ax.scatter(points[-1][0], points[-1][2], marker="s", color=color, s=24, zorder=7)


def save(fig, path):
    fig.savefig(path, dpi=140, facecolor="white")
    plt.close(fig)


def main(before_path, after_path, before_routes, after_routes, output):
    output.mkdir(parents=True, exist_ok=True)
    _, before = repair.load(before_path)
    _, after = repair.load(after_path)
    points, forward, incoming = repair.graph.parse_enhanced_route_edges(iter(before))
    changes = repair.plan(before)
    old = {r["road"]: r for r in json.loads(before_routes.read_text())}
    new = {r["road"]: r for r in json.loads(after_routes.read_text())}
    for path in (before_routes, after_routes):
        (output / path.name).write_bytes(path.read_bytes())
    all_out = defaultdict(list)
    for row in before: all_out[int(row["fromIndex"])].append(row)
    terminals = sorted(k for k,p in points.items()
                       if not all_out[k] or (p["name"].endswith("-Out") and not forward.get(k)))
    repaired = {c["terminal"] for c in changes}
    questionable = {10255, 3193, 12828}
    audit = []
    for k in terminals:
        p = points[k]
        if k in repaired: status = "Demi-tour corrige en amont"
        elif k == 15651: status = "Voie unique : retour non etabli"
        elif k in questionable: status = "Sortie synthetique seule : terrain a verifier"
        elif k in {5225,5670}: status = "Raccord de pont manuel existant"
        elif k == 3779: status = "Demi-tour Road 213 deja autorise"
        else: status = "Fin de voie ; voie adjacente continue"
        audit.append(dict(waypoint=k, road=p["road"], x=p["x"], y=p["y"], z=p["z"], status=status))

    for change in changes:
        road = int(change["road"])
        stop = change["stop"]
        before_result, after_result = old[road], new[road]
        fig,axes = plt.subplots(1,2,figsize=(12,6),layout="constrained")
        for ax, rows, result, label, color in zip(axes,[before,after],[before_result,after_result],["AVANT","APRES"],[ORANGE,GREEN]):
            backdrop(ax,rows,stop,40)
            route(ax,result,color)
            ax.scatter(*stop,marker="x",color=RED,s=65,zorder=8)
            ax.annotate(str(change["terminal"]),stop,xytext=(5,-13),textcoords="offset points",fontsize=8)
            status = f"trajet calcule : {result['length']:.1f} m" if result["found"] else "aucun trajet de retour"
            ax.set_title(f"{label} — {status}",fontsize=12)
        # Draw the removed tail and the exact Bezier turn, even if the route
        # simplifier drops samples. Purple/blue lines remain separate elevations.
        for row in change["removed"]:
            line=segment(row)
            axes[0].plot(*zip(*line),color=RED,lw=2.6)
        line=segment(change["row"])
        axes[1].plot(*zip(*line),color=GREEN,lw=4,zorder=8)
        normal=change["normal"]; tangent=(-normal[1],normal[0])
        plane=[(stop[0]+t*tangent[0],stop[1]+t*tangent[1]) for t in [-15,15]]
        for ax in axes: ax.plot(*zip(*plane),color=RED,lw=1,ls="--")
        fig.suptitle(f"Road {road} — demi-tour {change['start']} → {change['end']}\n"
                     "Meme cadrage / echelle ; croix = ancien terminus ; pointille = limite geometrique, pas un mur releve",fontsize=12)
        fig.supxlabel("Gris : graphe natif | bleu : altitude > 3 m | beige : liens synthetiques | orange/vert : trajet C#\n"
                       "Vue de dessus X/Z. Aucun test de collision en jeu. Depart rond / arrivee carree.",fontsize=9)
        save(fig,output/f"road-{road}-before-after.png")
        if before_result["found"]:
            path=before_result["points"]
            minx,maxx=min(p[0] for p in path),max(p[0] for p in path)
            minz,maxz=min(p[2] for p in path),max(p[2] for p in path)
            center=((minx+maxx)/2,(minz+maxz)/2)
            fig,ax=plt.subplots(figsize=(9,7),layout="constrained")
            backdrop(ax,before,center,max(maxx-minx,maxz-minz)/2+35)
            route(ax,before_result,ORANGE);route(ax,after_result,GREEN)
            ax.set_title(f"Road {road} — trajets complets, memes points de depart/arrivee\n"
                         f"Avant (orange) {before_result['length']:.1f} m ; apres (vert) {after_result['length']:.1f} m")
            save(fig,output/f"road-{road}-full-detour.png")

    unchanged=[r for r in audit if r["waypoint"] not in repaired]
    for page,offset in enumerate(range(0,len(unchanged),6),1):
        fig,axes=plt.subplots(2,3,figsize=(15,10),layout="constrained")
        for ax,item in zip(axes.flat,unchanged[offset:offset+6]):
            center=item["x"],item["z"]
            backdrop(ax,before,center,32)
            ax.scatter(*center,color=RED,marker="x",s=65,zorder=8)
            ax.set_title(f"R{item['road']} / wp {item['waypoint']}\n{item['status']}",fontsize=9)
        for ax in list(axes.flat)[len(unchanged[offset:offset+6]):]:ax.axis("off")
        fig.suptitle("Points audites sans modification : AVANT = APRES\n"
                     "Une fin de voie n'est pas necessairement une impasse ; aucun raccord invente vers une route voisine",fontsize=13)
        save(fig,output/f"unchanged-{page}.png")

    fig,ax=plt.subplots(figsize=(14,10),layout="constrained")
    backdrop(ax,before,(-1450,-650),2000)
    for n,item in enumerate(audit,1):
        color=GREEN if item["waypoint"] in repaired else RED
        ax.scatter(item["x"],item["z"],s=24,color=color,zorder=5)
        ax.annotate(str(n),(item["x"],item["z"]),xytext=(3,3+(n%3)*6),textcoords="offset points",fontsize=8)
    ax.set_title(f"Inventaire exhaustif des {len(audit)} terminus candidats du CSV\nVert : 6 corrections ; rouge : points conserves, voir tableau",fontsize=14)
    save(fig,output/"overview.png")
    (output/"audit.json").write_text(json.dumps(dict(
        before_sha256=hashlib.sha256(before_path.read_bytes()).hexdigest(),
        after_sha256=hashlib.sha256(after_path.read_bytes()).hexdigest(),
        points=audit, repairs=changes),indent=2),encoding="utf-8")
    lines=["# Impasses : comparaison du graphe avant/apres", "",
           "Le commentaire ne donne ni adresse ni capture. Les lieux ci-dessous sont des candidats, pas une reproduction confirmee du mur du pont.", "",
           "Preuve : donnees du graphe livre et polylignes calculees par le pathfinding C#. Ces dessins ne sont pas des captures du jeu et ne contiennent pas les collisions, trottoirs, murs ou rayons de braquage des vehicules.", "",
           "![Inventaire](overview.png)", "", "## Six corrections", "",
           "Les virages restent sur la meme route et au meme niveau. Leur enveloppe est au moins 2 m en amont du plan passant par l'ancien terminus. Les 14 points de fin de voie d'approche sont retires pour ne plus servir de candidats de navigation apres le virage. Quinze aretes natives sont remplacees par six demi-tours autorises.", "",
           "Road 246 avait deja une liaison native Waypoint -> Waypoint, mais elle n'etait pas reconnue comme demi-tour autorise. Les cinq autres terminus n'avaient aucune sortie directe.", "",
           "R237, R222 et R233 sont des terminus explicites Out de voie interieure : une autre voie continue vers un raccord. Ils ne prouvent donc pas que toute la rue est physiquement bouchee. La correction autorise le retour sur la meme route depuis cette voie, sans inventer un passage vers le raccord voisin. Les fins intermediaires Waypoint et les voies exterieures restent distinctes dans l'audit.", "",
           "| Route | Avant : longueur de polyligne | Apres | Demi-tour |", "|---|---:|---:|---|"]
    for c in changes:
        road=int(c["road"])
        distance=f"{old[road]['length']:.1f} m" if old[road]["found"] else "Aucun trajet"
        lines.append(f"| {road} | {distance} | {new[road]['length']:.1f} m | {c['start']} → {c['end']} |")
    for c in changes:
        road=int(c["road"])
        lines += ["",f"### Road {road}","",f"![Road {road} avant/apres](road-{road}-before-after.png)"]
        if old[road]["found"]:lines += ["",f"![Detour complet Road {road}](road-{road}-full-detour.png)"]
    lines += ["", "## Tous les candidats, y compris sans modification", "",
              "Les dix fins de voie dont une voie adjacente continue ne justifient pas un demi-tour a travers les voies. Road 169, d'abord suspectee pres du pont, appartient a ce groupe. Road 235 n'a pas de voie retour identifiee : en inventer une ferait circuler a contresens. Les sorties synthetiques de R222 (10255), R233 (3193) et R214 (12828) restent a verifier sur le terrain ; leur absence du graphe natif ne suffit pas a prouver un mur.", "",
              "| Repere | Road / waypoint | X, Y, Z (m) | Classement |", "|---|---|---|---|"]
    for i,item in enumerate(audit,1):
        lines.append(f"| {i} | R{item['road']} / {item['waypoint']} | {item['x']:.3f}, {item['y']:.3f}, {item['z']:.3f} | {item['status']} |")
    for page in range(1,math.ceil(len(unchanged)/6)+1):
        lines += ["",f"![Avant = apres, page {page}](unchanged-{page}.png)"]
    lines += ["", "## Reproduction", "",
              "Baseline PathFinding : `793a005fad5989d09e5713a7cbaa6d9d82310a9a`. Extraire son `data/big_ambitions_enhanced_routes.csv` vers `before.csv`.", "",
              "```text", "python tools/repair_deadend_turns.py data/big_ambitions_enhanced_routes.csv",
              "dotnet run --project DiagRunner -c Release -- before.csv --scenario deadends --output before-routes.json",
              "dotnet run --project DiagRunner -c Release -- data/big_ambitions_enhanced_routes.csv --scenario deadends --output after-routes.json",
              "python tools/render_deadend_review.py before.csv data/big_ambitions_enhanced_routes.csv before-routes.json after-routes.json docs/navigation-deadends", "```", "",
              "Les exports JSON contiennent les vrais indices A* et les points de la polyligne. Les images locales utilisent exactement le meme cadrage avant/apres ; les deux longs detours ont aussi une vue integrale.", ""]
    (output/"README.md").write_text("\n".join(lines),encoding="utf-8")
    print(f"Rendered {len(changes)} repairs and {len(unchanged)} unchanged candidates to {output}")


if __name__=="__main__":
    main(*(Path(arg) for arg in sys.argv[1:]))
