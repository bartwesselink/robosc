import { ComponentFixture, TestBed } from '@angular/core/testing';

import { PreliminariesRosServicesComponent } from './preliminaries-ros-services.component';

describe('PreliminariesRosServicesComponent', () => {
  let component: PreliminariesRosServicesComponent;
  let fixture: ComponentFixture<PreliminariesRosServicesComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ PreliminariesRosServicesComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(PreliminariesRosServicesComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
