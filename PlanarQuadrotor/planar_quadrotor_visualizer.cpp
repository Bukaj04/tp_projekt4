#include "planar_quadrotor_visualizer.h"
#include <cmath>

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers
 */

// Funkcja do rysowania wypełnionej, obróconej elipsy
void rysuj_elipse(SDL_Renderer* renderer, int x, int y, int rx, int ry, float angle, SDL_Color color) {
    const int segments = 100; // Możesz dostosować liczbę segmentów dla lepszej jakości wypełnienia
    float theta = 0.0;
    float delta_theta = 2.0 * M_PI / segments;
    SDL_Point points[segments + 1]; // Dodatkowy punkt dla środka elipsy

    points[0] = { x, y }; // Środek elipsy
    for (int i = 1; i <= segments; ++i) {
        float px = rx * cos(theta);
        float py = ry * sin(theta);

        float rotated_x = px * cos(angle) - py * sin(angle);
        float rotated_y = px * sin(angle) + py * cos(angle);

        points[i].x = x + rotated_x;
        points[i].y = y + rotated_y;
        theta += delta_theta;
    }

    // Wypełnij elipsę
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (int i = 1; i < segments; ++i) {
        SDL_RenderDrawLine(renderer, points[0].x, points[0].y, points[i].x, points[i].y);
        SDL_RenderDrawLine(renderer, points[i].x, points[i].y, points[i + 1].x, points[i + 1].y);
        SDL_RenderDrawLine(renderer, points[i + 1].x, points[i + 1].y, points[0].x, points[0].y);
    }
    // Zamknij ostatni segment
    SDL_RenderDrawLine(renderer, points[0].x, points[0].y, points[segments].x, points[segments].y);
    SDL_RenderDrawLine(renderer, points[segments].x, points[segments].y, points[1].x, points[1].y);
    SDL_RenderDrawLine(renderer, points[1].x, points[1].y, points[0].x, points[0].y);
}



PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    // x, y, theta współrzędne
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    // Pszesuń na środek
    int szerokosc_ekranu = 1280; 
    int wysokosc_ekranu = 780; 
    int srodek_x = szerokosc_ekranu / 2;
    int srodek_y = wysokosc_ekranu / 2;

    float obraz_x = srodek_x + q_x;
    float obraz_y = srodek_y - q_y; // Odwróć oś y dla reprezentacji graficznej

    // Wyczyść ekran
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF); // Białe tło
    SDL_RenderClear(gRenderer.get());

    // Narysuj prostokąt 
    SDL_SetRenderDrawColor(gRenderer.get(), 0xC0, 0xC0, 0xC0, 0xFF); // Czarny kolor
    SDL_Rect dronRect;
    dronRect.w = 120; // Szerokość prostokąta
    dronRect.h = 20; // Wysokość prostokąta
    dronRect.x = obraz_x - dronRect.w / 2; // Współrzędna x lewego górnego rogu prostokąta
    dronRect.y = obraz_y - dronRect.h / 2; // Współrzędna y lewego górnego rogu
    SDL_RenderFillRect(gRenderer.get(), &dronRect);

    int gorna_krawedz_y = obraz_y - dronRect.h / 2;
    int gorna_krawedz_x = obraz_x;

    // Wymiary połączeń
    int szerokosc_poloczenia = 10;
    int wysokosc_poloczenia = 40;

    // Środki śmigieł
    int smiglo1_srodek_x = gorna_krawedz_x - dronRect.w / 2 + szerokosc_poloczenia / 2;
    int smiglo1_srodek_y = gorna_krawedz_y - wysokosc_poloczenia;

    int smiglo2_srodek_x = gorna_krawedz_x + dronRect.w / 2 - szerokosc_poloczenia / 2;
    int smiglo2_srodek_y = gorna_krawedz_y - wysokosc_poloczenia;

    SDL_SetRenderDrawColor(gRenderer.get(), 0xA5, 0x2A, 0x2A, 0xFF);
    // Rysowanie połączenia śmigła lewego z dronem
    SDL_Rect polaczenie1;
    polaczenie1.x = gorna_krawedz_x - dronRect.w / 2;
    polaczenie1.y = gorna_krawedz_y - wysokosc_poloczenia;
    polaczenie1.w = szerokosc_poloczenia;
    polaczenie1.h = wysokosc_poloczenia;
    SDL_RenderFillRect(gRenderer.get(), &polaczenie1);

    // Rysowanie połączenia śmigła prawego z dronem 
    SDL_Rect polaczenie2;
    polaczenie2.x = gorna_krawedz_x + dronRect.w / 2 - szerokosc_poloczenia;
    polaczenie2.y = gorna_krawedz_y - wysokosc_poloczenia;
    polaczenie2.w = szerokosc_poloczenia;
    polaczenie2.h = wysokosc_poloczenia;
    SDL_RenderFillRect(gRenderer.get(), &polaczenie2);

    // Animacja śmigieł
    float czas = SDL_GetTicks() / 100.0f; // Pobierz czas w sekundach

    // Rysowanie śmigieł jako wypełnione elipsy
    SDL_Color darkBlue = { 0x00, 0x00, 0xFF, 0xFF }; // Ciemnoniebieski kolor
    SDL_Color lightBlue = { 0x87, 0xCE, 0xFA, 0xFF }; // Jasnoniebieski kolor
    rysuj_elipse(gRenderer.get(), smiglo1_srodek_x, smiglo1_srodek_y, 15, 4, czas, darkBlue);
    rysuj_elipse(gRenderer.get(), smiglo1_srodek_x, smiglo1_srodek_y, 15, 4, -czas + M_PI / 3, lightBlue);

    rysuj_elipse(gRenderer.get(), smiglo2_srodek_x, smiglo2_srodek_y, 15, 4, czas, darkBlue);
    rysuj_elipse(gRenderer.get(), smiglo2_srodek_x, smiglo2_srodek_y, 15, 4, -czas + M_PI / 3, lightBlue);

    // Prezentuj narysowaną klatkę
    SDL_RenderPresent(gRenderer.get());
    SDL_Delay(8);
}
