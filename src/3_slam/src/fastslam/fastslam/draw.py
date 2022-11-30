import matplotlib.pyplot as plt

def draw_animation(RFID, N_PARTICLE, hxTrue, hxDR, hxEst, z, xEst, particles):
    plt.cla()
# for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])
    plt.plot(RFID[:, 0], RFID[:, 1], "*k")

    for iz in range(len(z[:, 0])):
        landmark_id = int(z[2, iz])
        plt.plot([xEst[0], RFID[landmark_id, 0]], [
            xEst[1], RFID[landmark_id, 1]], "-k")

    for i in range(N_PARTICLE):
        plt.plot(particles[i].x, particles[i].y, ".r")
        plt.plot(particles[i].lm[:, 0], particles[i].lm[:, 1], "xb")

    plt.plot(hxTrue[0, :], hxTrue[1, :], "-b")
    plt.plot(hxDR[0, :], hxDR[1, :], "-k")
    plt.plot(hxEst[0, :], hxEst[1, :], "-r")
    plt.plot(xEst[0], xEst[1], "xk")
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.001)
