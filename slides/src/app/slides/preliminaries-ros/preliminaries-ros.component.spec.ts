import { ComponentFixture, TestBed } from '@angular/core/testing';

import { PreliminariesRosComponent } from './preliminaries-ros.component';

describe('PreliminariesRosComponent', () => {
  let component: PreliminariesRosComponent;
  let fixture: ComponentFixture<PreliminariesRosComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ PreliminariesRosComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(PreliminariesRosComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
