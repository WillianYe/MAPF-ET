from Roadmap import Roadmap
import argparse


def read_scen(map_tag, num):
    for i in range(num):
        scen_file = f'./benchmark/mapf-scens/{map_tag}/{map_tag}-random-{i+1}.scen'
        scen_file2 = f'./benchmark/mapf-dt-scens/{map_tag}/{map_tag}-tc-random-{i + 1}.scen'

        map_ls = open(scen_file, 'r').readlines()

        scene_data = [elem.replace('\n', '') for elem in map_ls]
        scene_data.pop(0)
        height = len(scene_data)
        entries = {}
        new_entries = {}
        for i in range(height):
            entries[i] = []
        for i in range(height):
            entries[i] = scene_data[i].split("\t")
        width = len(entries[0])
        print(height)
        print(width)

        file = open(scen_file2, "w")

        file.write("version 1")
        file.write("\n")

        for r in range(height):
            r1 = int(entries[r][5])
            c1 = int(entries[r][4])
            r2 = int(entries[r][7])
            c2 = int(entries[r][6])         

            # node1 = ln.xy2node[r1][c1]
            # node2 = ln.xy2node[r2][c2]
            node1 = ln.grid2node[r1,c1]
            node2 = ln.grid2node[r2,c2]
            entries[r].append(str(ln.dijkstra(node1)[node2]))
            for c in entries[r]:
                file.write(c)
                file.write("\t")
            file.write("\n")
        file.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", type=str, help="select map dir")
    parser.add_argument("tag", type=str, help="select map tag")
    args = parser.parse_args()
    ln = Roadmap(args.map_dir)
    read_scen(args.tag, 25)
